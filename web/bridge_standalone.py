#!/usr/bin/env python3
"""
SO-101 Robotics Standalone Hardware Bridge (Cross-Platform: Windows, Linux, macOS)
Zero ROS 2 dependencies required! Pure Python + pyserial + tornado + opencv.

Provides:
- Synchronous STS3215 raw serial communication driver with SYNC_WRITE (0x83)
- Real-time WebSockets telemetry on ws://localhost:8080/ws
- Complete Camera Streaming (RGB, Depth false-color, RGB+Depth) on /api/camera/stream
- Live trajectory CSV recording and dataset playback
- Web UI serving on http://localhost:8080
"""

import os
import sys
import json
import time
import math
import csv
import glob
import webbrowser
import threading
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Set, Optional, Any

ROOT_DIR = Path(__file__).resolve().parent


def _ensure_dependencies():
    """Install missing pip packages for THIS interpreter (venv-aware, PEP 668-aware)."""
    needed = {"serial": "pyserial", "tornado": "tornado", "cv2": "opencv-python", "numpy": "numpy"}
    import importlib.util
    missing = [pkg for mod, pkg in needed.items() if importlib.util.find_spec(mod) is None]
    if not missing:
        return
    import subprocess
    print(f"\n[*] Installing missing packages: {', '.join(missing)} ...")
    cmd = [sys.executable, "-m", "pip", "install", *missing]
    rc = subprocess.call(cmd)
    if rc != 0:  # Debian 12 / Ubuntu 24.04 / Fedora 38+ refuse system-wide installs (PEP 668)
        rc = subprocess.call(cmd + ["--break-system-packages"])
    if rc != 0:
        print("\n[ERROR] Could not install: " + " ".join(missing))
        print("        Use the launcher (run_linux.sh / run_windows.bat) which creates a private .venv,")
        print("        or run:  python3 -m venv .venv && .venv/bin/pip install -r requirements-bridge.txt")
        sys.exit(1)


_ensure_dependencies()

import serial
import serial.tools.list_ports
import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.gen

# camera_backend.py lives next to this file (desktop package) or in the ROS package (repo checkout)
for _cand in (ROOT_DIR, ROOT_DIR / "src" / "so101_servo_dynamics" / "so101_servo_dynamics"):
    if (_cand / "camera_backend.py").is_file():
        sys.path.insert(0, str(_cand))
        break
import camera_backend  # noqa: E402
from camera_backend import CameraManager, CV2_AVAILABLE, REALSENSE_AVAILABLE, ORBBEC_AVAILABLE  # noqa: E402


def _resolve_web_dir() -> Path:
    """Works for repo layout (./web/index.html) AND flat desktop package (./index.html)."""
    for cand in (ROOT_DIR / "web", ROOT_DIR):
        if (cand / "index.html").is_file():
            return cand
    return ROOT_DIR / "web"


WEB_DIR = _resolve_web_dir()
DATA_DIR = ROOT_DIR / "data" / "raw"
DATA_DIR.mkdir(parents=True, exist_ok=True)

# Default STS3215 joint calibration
DEFAULT_CALIBRATION = {
    1: {"name": "shoulder_pan",  "id": 1, "homing_offset": 1823, "min": 1197, "max": 3113, "rad_min": -1.4678, "rad_max": 1.4678, "deg_min": -84.1, "deg_max": 84.1},
    2: {"name": "shoulder_lift", "id": 2, "homing_offset": 985,  "min": 1,    "max": 4091, "rad_min": -3.1398, "rad_max": 3.1398, "deg_min": -179.9, "deg_max": 179.9},
    3: {"name": "elbow_flex",    "id": 3, "homing_offset": -327, "min": 0,    "max": 4094, "rad_min": -3.1416, "rad_max": 3.1398, "deg_min": -180.0, "deg_max": 179.9},
    4: {"name": "wrist_flex",    "id": 4, "homing_offset": 1874, "min": 1102, "max": 3022, "rad_min": -1.4713, "rad_max": 1.4713, "deg_min": -84.3, "deg_max": 84.3},
    5: {"name": "wrist_roll",    "id": 5, "homing_offset": 601,  "min": 0,    "max": 4095, "rad_min": -3.1416, "rad_max": 3.1416, "deg_min": -180.0, "deg_max": 180.0},
    6: {"name": "gripper",       "id": 6, "homing_offset": 1467, "min": 2013, "max": 3505, "rad_min": 0.0,    "rad_max": 1.0,    "deg_min": 0.0,   "deg_max": 100.0},
}

STS_ENCODER_RESOLUTION = 4096


def calc_sts_checksum(pkt: bytearray) -> int:
    return (~sum(pkt[2:])) & 0xFF


# USB-serial bridge chips used by Feetech/Waveshare servo boards
_PREFERRED_VIDS = {0x1A86, 0x10C4, 0x0403, 0x2E8A, 0x303A}   # CH340/CH343, CP210x, FTDI, RP2040, ESP32-S3


_JUNK_PORTS: Set[str] = set()   # Bluetooth / onboard UARTs: listed, but never auto-connected


def list_available_serial_ports() -> List[str]:
    """Serial ports on Windows (COM*), Linux (ttyACM/ttyUSB) and macOS (cu.usb*).
    USB adapters first; Bluetooth / legacy onboard ports last so auto-pick never grabs them."""
    ranked = []
    try:
        for p in serial.tools.list_ports.comports():
            desc = f"{p.description or ''} {p.hwid or ''}".lower()
            usb = p.vid in _PREFERRED_VIDS
            junk = "bluetooth" in desc or "bthenum" in desc or p.device.startswith(("/dev/ttyS", "/dev/ttyAMA"))
            if junk:
                _JUNK_PORTS.add(p.device)
            ranked.append((0 if usb else (2 if junk else 1), p.device))
    except Exception:
        pass
    if not ranked:
        if sys.platform.startswith("win"):
            return []   # blind COM1..32 guesses made auto-connect pick non-existent ports
        for pat in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usb*"):
            ranked += [(0, d) for d in glob.glob(pat)]
    ranked.sort()
    seen, out = set(), []
    for _, dev in ranked:
        if dev not in seen:
            seen.add(dev); out.append(dev)
    return out


def serial_permission_hint(port: str) -> str:
    if sys.platform.startswith("linux") and port and os.path.exists(port) and not os.access(port, os.R_OK | os.W_OK):
        return (f"No permission for {port}. Add your user to the serial group "
                "(Ubuntu/Debian/Fedora: dialout, Arch: uucp) then log out/in: "
                "sudo usermod -aG dialout $USER")
    return ""


def load_home_position() -> Dict[str, float]:
    home_file = ROOT_DIR / "home_position.txt"
    pos = {
        "shoulder_pan": 0.88,
        "shoulder_lift": -104.62,
        "elbow_flex": 100.57,
        "wrist_flex": 59.96,
        "wrist_roll": 1.71,
        "gripper": 3.22
    }
    if home_file.is_file():
        try:
            for line in home_file.read_text().splitlines():
                if ":" in line:
                    k, v = line.split(":", 1)
                    k = k.strip().replace(".pos", "")
                    pos[k] = float(v.strip())
        except Exception:
            pass
    return pos


def load_zero_position() -> Dict[str, float]:
    return {
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
        "gripper": 0.0
    }


class StandaloneSTS3215Driver:
    """Synchronous Feetech STS3215 raw serial communication driver."""
    def __init__(self, port: str = "", baudrate: int = 1000000, timeout: float = 0.04):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_connected = False
        self._ser: Optional[serial.Serial] = None
        self.calibration = DEFAULT_CALIBRATION
        self.lock = threading.Lock()

    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None) -> bool:
        with self.lock:
            if port: self.port = port
            if baudrate: self.baudrate = baudrate
            self.disconnect()
            try:
                self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
                self.is_connected = True
                print(f"[OK] Connected to hardware on {self.port} at {self.baudrate} baud.")
                self._sync_write_torque_locked(True)
                return True
            except Exception as e:
                print(f"[WARN] Could not connect to {self.port}: {e}")
                hint = serial_permission_hint(self.port)
                if hint: print(f"[HINT] {hint}")
                self.is_connected = False
                return False

    def disconnect(self):
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.is_connected = False

    def _sync_write_torque_locked(self, enable: bool):
        if not self._ser or not self._ser.is_open: return
        val = 1 if enable else 0
        pkt = bytearray([0xFF, 0xFF, 0xFE, 16, 0x83, 40, 1])
        for sid in range(1, 7):
            pkt.append(sid)
            pkt.append(val)
        pkt.append(calc_sts_checksum(pkt))
        try:
            self._ser.write(pkt)
        except Exception:
            pass

    def set_torque(self, enable: bool) -> bool:
        with self.lock:
            self._sync_write_torque_locked(enable)
            return True

    def ticks_to_angles(self, sid: int, ticks: int):
        cal = self.calibration.get(sid, {})
        c_min = cal.get("min", 0)
        c_max = cal.get("max", 4095)
        if sid == 6:
            pct = ((ticks - c_min) / max(1, c_max - c_min)) * 100.0
            pct = max(0.0, min(100.0, pct))
            return pct / 100.0, pct
        mid = (c_min + c_max) / 2.0
        deg = (ticks - mid) * (360.0 / 4095.0)
        return math.radians(deg), deg

    def deg_to_ticks(self, sid: int, deg: float) -> int:
        cal = self.calibration.get(sid, {})
        c_min = cal.get("min", 0)
        c_max = cal.get("max", 4095)
        if sid == 6:
            pct = max(0.0, min(100.0, deg))
            ticks = int((pct / 100.0) * (c_max - c_min) + c_min)
            return max(c_min, min(c_max, ticks))
        mid = (c_min + c_max) / 2.0
        ticks = int((deg * 4095.0 / 360.0) + mid)
        return max(c_min, min(c_max, ticks))

    def read_all_servos(self) -> Dict[int, Dict[str, Any]]:
        result = {}
        if not self.is_connected or not self._ser:
            return result

        with self.lock:
            for sid, cal in self.calibration.items():
                try:
                    pkt = bytearray([0xFF, 0xFF, sid, 4, 0x02, 56, 8])
                    pkt.append(calc_sts_checksum(pkt))
                    self._ser.reset_input_buffer()
                    self._ser.write(pkt)
                    resp = self._ser.read(14)
                    if len(resp) >= 14 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == sid:
                        data = resp[5:13]
                        pos_raw = int.from_bytes(data[0:2], "little", signed=True)
                        vel_raw = int.from_bytes(data[2:4], "little", signed=True)
                        volt = data[6] / 10.0
                        temp = data[7]
                        pos_rad, pos_deg = self.ticks_to_angles(sid, pos_raw)

                        result[sid] = {
                            "id": sid,
                            "joint_name": cal["name"],
                            "encoder_ticks": pos_raw,
                            "position_rad": pos_rad,
                            "position_deg": pos_deg,
                            "velocity_raw": vel_raw,
                            "velocity_rad_s": vel_raw * (2.0 * math.pi / STS_ENCODER_RESOLUTION),
                            "voltage_v": volt,
                            "temperature_c": temp,
                            "status": 0,
                            "error_alarm": False
                        }
                except Exception:
                    pass
        return result

    def sync_write_all(self, positions_deg: Dict[str, float], speed_rad_s: float = 1.5) -> bool:
        if not self.is_connected or not self._ser:
            return False

        speed_ticks = max(10, min(3400, int(speed_rad_s * STS_ENCODER_RESOLUTION / (2.0 * math.pi))))
        with self.lock:
            pkt = bytearray([0xFF, 0xFF, 0xFE, 46, 0x83, 42, 6])
            for sid in range(1, 7):
                cal = self.calibration[sid]
                name = cal["name"]
                deg = positions_deg.get(name, 0.0)
                ticks = self.deg_to_ticks(sid, deg)
                pkt.append(sid)
                pkt.append(ticks & 0xFF)
                pkt.append((ticks >> 8) & 0xFF)
                pkt.append(0)  # Time Low
                pkt.append(0)  # Time High
                pkt.append(speed_ticks & 0xFF)
                pkt.append((speed_ticks >> 8) & 0xFF)

            pkt.append(calc_sts_checksum(pkt))
            try:
                self._ser.write(pkt)
                return True
            except Exception as e:
                print(f"[ERROR] Sync write failed: {e}")
                return False


driver = StandaloneSTS3215Driver()
camera_manager = CameraManager()
connected_websockets: Set[tornado.websocket.WebSocketHandler] = set()
active_targets: Dict[str, float] = {}
is_recording = False
record_file = None
record_writer = None
sim_positions: Dict[str, float] = {}


class BaseApiHandler(tornado.web.RequestHandler):
    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization, X-Requested-With")
        self.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        # Chrome/Edge Private Network Access: public HTTPS page (Render) -> localhost bridge
        self.set_header("Access-Control-Allow-Private-Network", "true")

    def options(self, *args, **kwargs):
        self.set_status(204)
        self.finish()


class CameraDevicesHandler(BaseApiHandler):
    async def get(self):
        force = self.get_argument("refresh", "0") == "1"
        loop = tornado.ioloop.IOLoop.current()
        devices = await loop.run_in_executor(None, camera_manager.list_devices, force)
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps({"success": True, "devices": devices,
                               "diagnostics": camera_manager.diagnostics()}))


class CameraConnectHandler(BaseApiHandler):
    async def post(self):
        self.set_header("Content-Type", "application/json")
        try:
            data = json.loads(self.request.body)
            args = (str(data.get("device_id", "0")), data.get("camera_type", "usb"),
                    int(data.get("width", 640)), int(data.get("height", 480)),
                    int(data.get("fps", 30)), data.get("custom_url"))
            loop = tornado.ioloop.IOLoop.current()
            res = await loop.run_in_executor(None, lambda: camera_manager.connect(*args))
            self.write(json.dumps(res))
        except Exception as e:
            self.write(json.dumps({"success": False, "message": str(e)}))


class CameraDisconnectHandler(BaseApiHandler):
    async def post(self):
        loop = tornado.ioloop.IOLoop.current()
        await loop.run_in_executor(None, camera_manager.disconnect)
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps({"success": True, "message": "Camera disconnected."}))


class CameraStatusHandler(BaseApiHandler):
    def get(self):
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(camera_manager.get_status()))


class CameraSettingsHandler(BaseApiHandler):
    def post(self):
        self.set_header("Content-Type", "application/json")
        try:
            data = json.loads(self.request.body)
            if "brightness" in data: camera_manager.brightness = int(data["brightness"])
            if "contrast" in data: camera_manager.contrast = int(data["contrast"])
            if "stream_mode" in data: camera_manager.stream_mode = str(data["stream_mode"])
            self.write(json.dumps({"success": True, "status": camera_manager.get_status()}))
        except Exception as e:
            self.write(json.dumps({"success": False, "message": str(e)}))


class CameraSnapshotHandler(BaseApiHandler):
    def get(self):
        mode = self.get_argument("mode", camera_manager.stream_mode or "rgb")
        frame_bytes = camera_manager.get_jpeg_frame(mode=mode)
        if frame_bytes:
            self.set_header("Content-Type", "image/jpeg")
            self.set_header("Content-Disposition", f"attachment; filename=snapshot_{int(time.time())}.jpg")
            self.write(frame_bytes)
        else:
            self.set_status(404)
            self.write("No frame available.")


class CameraStreamHandler(BaseApiHandler):
    def initialize(self):
        self._is_active = True

    def on_connection_close(self):
        self._is_active = False

    @tornado.gen.coroutine
    def get(self):
        mode = self.get_argument("mode", camera_manager.stream_mode or "rgb")
        self.set_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.set_header("Cache-Control", "no-cache, no-store, must-revalidate")
        last_seq = -1
        while self._is_active:
            if not camera_manager.is_connected:
                frame_bytes = camera_manager.get_placeholder_jpeg("Camera Standby - Click Connect")
                last_seq = -1
            elif camera_manager.frame_seq == last_seq:
                yield tornado.gen.sleep(0.01)      # nothing new: don't re-send / re-encode
                continue
            else:
                last_seq = camera_manager.frame_seq
                frame_bytes = camera_manager.get_jpeg_frame(mode=mode)
            if frame_bytes:
                try:
                    self.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                               + f"Content-Length: {len(frame_bytes)}\r\n\r\n".encode()
                               + frame_bytes + b"\r\n")
                    yield self.flush()
                except Exception:
                    break
            yield tornado.gen.sleep(0.5 if not camera_manager.is_connected else 0.005)


class SystemInfoHandler(BaseApiHandler):
    """Used by the UI and by acceptance tests to show exactly what this machine supports."""
    def get(self):
        self.set_header("Content-Type", "application/json")
        ports = list_available_serial_ports()
        self.write(json.dumps({
            "ok": True, "bridge": "standalone", "web_dir": str(WEB_DIR),
            "serial_ports": ports, "serial_active": driver.port, "serial_connected": driver.is_connected,
            "camera": camera_manager.diagnostics(),
        }))


class WebSocketHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        return True

    def open(self):
        connected_websockets.add(self)
        available_ports = list_available_serial_ports()
        self.write_message(json.dumps({
            "type": "init",
            "calibration": DEFAULT_CALIBRATION,
            "home_position": load_home_position(),
            "zero_position": load_zero_position(),
            "available_ports": available_ports,
            "active_port": driver.port,
            "robot_connected": driver.is_connected,
            "online_count": 6 if driver.is_connected else 0,
            "telemetry": {}
        }))

    def on_close(self):
        connected_websockets.discard(self)

    def on_message(self, message):
        global is_recording, record_file, record_writer
        try:
            data = json.loads(message)
            action = data.get("type")

            if action == "connect_robot":
                port = data.get("port", driver.port)
                baud = int(data.get("baudrate", 1000000))
                ok = driver.connect(port, baud)
                self.write_message(json.dumps({"type": "connect_result", "result": ok}))

            elif action == "disconnect_robot":
                driver.disconnect()
                self.write_message(json.dumps({"type": "disconnect_result", "result": True}))

            elif action == "torque":
                enable = bool(data.get("enable", True))
                ok = driver.set_torque(enable)
                self.write_message(json.dumps({"type": "torque_result", "enable": enable, "success": ok}))

            elif action == "refresh_ports":
                self.write_message(json.dumps({
                    "type": "ports_list",
                    "ports": list_available_serial_ports()
                }))

            elif action == "command":
                joint = data.get("joint")
                pos_deg = float(data.get("position_deg", 0.0))
                speed = float(data.get("speed", 1.5))
                active_targets[joint] = pos_deg
                if driver.is_connected:
                    driver.sync_write_all(active_targets, speed)
                else:
                    sim_positions[joint] = pos_deg

            elif action == "multi_command":
                pos_dict = data.get("positions", {})
                speed = float(data.get("speed", 1.5))
                for k, v in pos_dict.items():
                    active_targets[k] = float(v)
                if driver.is_connected:
                    driver.sync_write_all(active_targets, speed)
                else:
                    for k, v in pos_dict.items():
                        sim_positions[k] = float(v)

            elif action == "home":
                speed = float(data.get("speed", 1.2))
                home_pos = load_home_position()
                for k, v in home_pos.items():
                    active_targets[k] = float(v)
                if driver.is_connected:
                    driver.sync_write_all(active_targets, speed)
                else:
                    for k, v in home_pos.items():
                        sim_positions[k] = float(v)

            elif action == "zero":
                speed = float(data.get("speed", 1.2))
                zero_pos = load_zero_position()
                for k, v in zero_pos.items():
                    active_targets[k] = float(v)
                if driver.is_connected:
                    driver.sync_write_all(active_targets, speed)
                else:
                    for k, v in zero_pos.items():
                        sim_positions[k] = float(v)

            elif action == "start_record":
                prefix = data.get("prefix", "manual_recording")
                driver.set_torque(False)
                fname = f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
                fpath = DATA_DIR / fname
                record_file = open(fpath, "w", newline="")
                record_writer = csv.writer(record_file)
                record_writer.writerow(["timestamp", "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"])
                is_recording = True
                self.write_message(json.dumps({"type": "record_status", "recording": True, "filename": fname}))

            elif action == "stop_record":
                is_recording = False
                if record_file:
                    record_file.close()
                    record_file = None
                    record_writer = None
                driver.set_torque(True)
                self.write_message(json.dumps({"type": "record_status", "recording": False, "result": True}))

        except Exception as ex:
            print(f"[WARN] WebSocket message error: {ex}")


def poll_and_broadcast():
    """Periodically reads feedback from hardware and broadcasts telemetry over WebSocket."""
    global is_recording, record_writer
    feedback = {}
    if driver.is_connected:
        feedback = driver.read_all_servos()

    now = time.time()
    telemetry_joints = {}

    for sid, cal in DEFAULT_CALIBRATION.items():
        name = cal["name"]
        fb = feedback.get(sid)
        if fb:
            act_deg = fb["position_deg"]
            volt = fb["voltage_v"]
            temp = fb["temperature_c"]
            ticks = fb["encoder_ticks"]
        else:
            act_deg = sim_positions.get(name, 0.0)
            volt = 5.0
            temp = 28
            ticks = driver.deg_to_ticks(sid, act_deg)

        tgt_deg = active_targets.get(name, act_deg)
        err_deg = abs(tgt_deg - act_deg)

        telemetry_joints[name] = {
            "id": sid,
            "name": name,
            "target_deg": tgt_deg,
            "actual_deg": act_deg,
            "error_deg": err_deg,
            "reaction_time_ms": 18.0,
            "voltage_v": volt,
            "temperature_c": temp,
            "encoder_ticks": ticks
        }

    if is_recording and record_writer:
        try:
            row = [now] + [telemetry_joints[cal["name"]]["actual_deg"] for sid, cal in DEFAULT_CALIBRATION.items()]
            record_writer.writerow(row)
        except Exception:
            pass

    msg = json.dumps({
        "type": "telemetry",
        "data": {
            "timestamp": now,
            "robot_connected": driver.is_connected,
            "online_count": len(feedback) if driver.is_connected else 6,
            "port": driver.port,
            "joints": telemetry_joints
        }
    })

    dead = []
    for ws in connected_websockets:
        try:
            ws.write_message(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        connected_websockets.discard(ws)


def make_app():
    return tornado.web.Application([
        (r"/ws", WebSocketHandler),
        (r"/api/system/info", SystemInfoHandler),
        (r"/api/camera/devices", CameraDevicesHandler),
        (r"/api/camera/status", CameraStatusHandler),
        (r"/api/camera/connect", CameraConnectHandler),
        (r"/api/camera/disconnect", CameraDisconnectHandler),
        (r"/api/camera/settings", CameraSettingsHandler),
        (r"/api/camera/snapshot", CameraSnapshotHandler),
        (r"/api/camera/stream", CameraStreamHandler),
        (r"/(.*)", tornado.web.StaticFileHandler, {"path": str(WEB_DIR), "default_filename": "index.html"}),
    ])


def main():
    import argparse
    parser = argparse.ArgumentParser(description="SO-101 Robotics Standalone Bridge")
    parser.add_argument("--port", default=None, help="Serial port (e.g. COM3 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=1000000, help="Baud rate (default: 1000000)")
    parser.add_argument("--web-port", type=int, default=8080, help="Web server port (default: 8080)")
    parser.add_argument("--no-browser", action="store_true", help="Do not automatically open browser")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (use 127.0.0.1 to block LAN access)")
    parser.add_argument("--test-camera", action="store_true", help="Add a synthetic RGB+depth camera for self-tests")
    args = parser.parse_args()

    if args.test_camera:
        os.environ["SO101_TEST_CAMERA"] = "1"
    ports = list_available_serial_ports()
    target_port = args.port
    if not target_port and ports and ports[0] not in _JUNK_PORTS:
        target_port = ports[0]
    elif not target_port:
        target_port = None

    print("=================================================================")
    print("      SO-101 ROBOTICS STANDALONE HARDWARE BRIDGE (ZERO ROS 2)     ")
    print("=================================================================")
    print(f" Available ports : {ports if ports else 'None detected'}")
    print(f" Attempting port : {target_port or '(none - simulation)'} at {args.baud} baud")
    print(f" Web UI URL      : http://localhost:{args.web_port}")
    print(f" Web UI folder   : {WEB_DIR}")
    print(f" Camera support  : OpenCV={CV2_AVAILABLE}  RealSense={REALSENSE_AVAILABLE}  Orbbec={ORBBEC_AVAILABLE}")
    print("=================================================================\n")

    if target_port:
        driver.connect(target_port, args.baud)
    else:
        print("[WARN] No serial port detected - running in simulation mode. Plug in the arm and use 'Connect Robot'.")

    home = load_home_position()
    for k, v in home.items():
        active_targets[k] = v
        sim_positions[k] = v

    app = make_app()
    app.listen(args.web_port, address=args.host)

    loop = tornado.ioloop.IOLoop.current()
    periodic = tornado.ioloop.PeriodicCallback(poll_and_broadcast, 33)
    periodic.start()

    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{args.web_port}")).start()

    print(f"[*] Bridge server running on http://localhost:{args.web_port}. Press Ctrl+C to exit.")
    try:
        loop.start()
    except KeyboardInterrupt:
        print("\nStopping bridge server...")
        driver.disconnect()
        camera_manager.disconnect()
        loop.stop()


if __name__ == "__main__":
    main()
