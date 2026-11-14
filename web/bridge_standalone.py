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

import serial
import serial.tools.list_ports

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False

try:
    import tornado.ioloop
    import tornado.web
    import tornado.websocket
    import tornado.gen
except ImportError:
    print("\n[*] Installing required dependencies (tornado, pyserial, opencv-python)...")
    os.system(f"{sys.executable} -m pip install tornado pyserial opencv-python")
    import tornado.ioloop
    import tornado.web
    import tornado.websocket
    import tornado.gen

ROOT_DIR = Path(__file__).resolve().parent
WEB_DIR = ROOT_DIR / "web"
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


def list_available_serial_ports() -> List[str]:
    """Auto-detects serial ports across Windows (COM*), Linux, and macOS."""
    ports = []
    try:
        for p in serial.tools.list_ports.comports():
            ports.append(p.device)
    except Exception:
        pass

    if not ports:
        if sys.platform.startswith("win"):
            for i in range(1, 33):
                ports.append(f"COM{i}")
        else:
            ports = glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/cu.usb*")
    return sorted(list(set(ports)))


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
    def __init__(self, port: str = "COM3", baudrate: int = 1000000, timeout: float = 0.04):
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


class StandaloneCameraManager:
    """Manages OpenCV webcams and RealSense 3D cameras."""
    def __init__(self):
        self.is_connected = False
        self.camera_type = "usb"
        self.device_id = "/dev/video0" if not sys.platform.startswith("win") else "0"
        self.width = 640
        self.height = 480
        self.target_fps = 30
        self.stream_mode = "rgb"  # "rgb", "depth", "side_by_side"
        self.brightness = 50
        self.contrast = 50

        self.cap: Optional[Any] = None
        self.rs_pipeline = None
        self.rs_config = None
        self.is_running = False
        self.thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.latest_rgb: Optional[Any] = None
        self.latest_depth: Optional[Any] = None

    def list_devices(self) -> List[Dict[str, Any]]:
        devices = []
        if REALSENSE_AVAILABLE:
            try:
                ctx = rs.context()
                for dev in ctx.query_devices():
                    name = dev.get_info(rs.camera_info.name)
                    serial = dev.get_info(rs.camera_info.serial_number)
                    devices.append({
                        "id": f"realsense:{serial}",
                        "name": f"Intel RealSense {name} (S/N: {serial})",
                        "type": "realsense",
                        "has_rgb": True,
                        "has_depth": True,
                        "default_res": "640x480"
                    })
            except Exception:
                pass

        if CV2_AVAILABLE:
            if sys.platform.startswith("win"):
                for idx in range(3):
                    try:
                        c = cv2.VideoCapture(idx)
                        if c.isOpened():
                            c.release()
                            devices.append({
                                "id": str(idx),
                                "name": f"Camera {idx} (Windows UVC)",
                                "type": "usb",
                                "has_rgb": True,
                                "has_depth": False,
                                "default_res": "640x480"
                            })
                    except Exception:
                        pass
            else:
                for p in sorted(glob.glob("/dev/video*")):
                    try:
                        idx = int(p.replace("/dev/video", ""))
                        c = cv2.VideoCapture(idx)
                        if c.isOpened():
                            c.release()
                            devices.append({
                                "id": p,
                                "name": f"USB Video Device ({p})",
                                "type": "usb",
                                "has_rgb": True,
                                "has_depth": False,
                                "default_res": "640x480"
                            })
                    except Exception:
                        pass

        devices.append({"id": "orbbec_3d", "name": "Orbbec 3D Camera", "type": "orbbec", "has_rgb": True, "has_depth": True, "default_res": "640x480"})
        devices.append({"id": "network_esp32", "name": "ESP32-CAM / IP Stream", "type": "network", "has_rgb": True, "has_depth": False, "default_res": "640x480"})
        return devices

    def connect(self, device_id: str, camera_type: str = "usb", width: int = 640, height: int = 480, fps: int = 30, custom_url: Optional[str] = None):
        self.disconnect()
        self.device_id = device_id
        self.camera_type = camera_type
        self.width = width
        self.height = height
        self.target_fps = fps

        if not CV2_AVAILABLE:
            return {"success": False, "message": "OpenCV (cv2) is not installed."}

        try:
            if camera_type == "realsense" and REALSENSE_AVAILABLE:
                self.rs_pipeline = rs.pipeline()
                self.rs_config = rs.config()
                if ":" in device_id:
                    self.rs_config.enable_device(device_id.split(":")[1])
                self.rs_config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
                self.rs_config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
                self.rs_pipeline.start(self.rs_config)
            elif camera_type == "network":
                url = custom_url or "http://192.168.4.1/stream"
                self.cap = cv2.VideoCapture(url)
                if not self.cap.isOpened():
                    return {"success": False, "message": f"Could not connect to network stream: {url}"}
            else:
                idx = 0
                if device_id.startswith("/dev/video"):
                    idx = int(device_id.replace("/dev/video", ""))
                elif device_id.isdigit():
                    idx = int(device_id)
                self.cap = cv2.VideoCapture(idx)
                if not self.cap or not self.cap.isOpened():
                    return {"success": False, "message": f"Could not open camera device {device_id}"}
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                self.cap.set(cv2.CAP_PROP_FPS, fps)

            self.is_connected = True
            self.is_running = True
            self.thread = threading.Thread(target=self._capture_worker, daemon=True)
            self.thread.start()
            return {"success": True, "message": f"Connected to {camera_type} camera."}
        except Exception as e:
            return {"success": False, "message": str(e)}

    def disconnect(self):
        self.is_running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None

        if self.rs_pipeline:
            try: self.rs_pipeline.stop()
            except Exception: pass
            self.rs_pipeline = None

        if self.cap:
            try: self.cap.release()
            except Exception: pass
            self.cap = None

        self.is_connected = False
        with self.lock:
            self.latest_rgb = None
            self.latest_depth = None

    def _capture_worker(self):
        while self.is_running and self.is_connected:
            try:
                if self.rs_pipeline:
                    frames = self.rs_pipeline.wait_for_frames(timeout_ms=1000)
                    color_frame = frames.get_color_frame()
                    depth_frame = frames.get_depth_frame()
                    if color_frame and depth_frame:
                        rgb = np.asanyarray(color_frame.get_data())
                        depth_img = np.asanyarray(depth_frame.get_data())
                        depth_colored = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET)
                        with self.lock:
                            self.latest_rgb = rgb
                            self.latest_depth = depth_colored
                elif self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        # Generate pseudo-depth map using edge/intensity gradient
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        depth_sim = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
                        with self.lock:
                            self.latest_rgb = frame
                            self.latest_depth = depth_sim
                time.sleep(1.0 / max(10, self.target_fps))
            except Exception:
                time.sleep(0.05)

    def get_jpeg_frame(self, mode: str = "rgb") -> Optional[bytes]:
        with self.lock:
            if not self.is_connected or self.latest_rgb is None:
                # Placeholder frame
                img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(img, "Camera Standby - Click Connect", (120, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                _, enc = cv2.imencode(".jpg", img)
                return enc.tobytes()

            frame = self.latest_rgb
            if mode == "depth" and self.latest_depth is not None:
                frame = self.latest_depth
            elif mode == "side_by_side" and self.latest_depth is not None:
                h, w = frame.shape[:2]
                depth_resized = cv2.resize(self.latest_depth, (w, h))
                frame = np.hstack((frame, depth_resized))

            # Apply tuning
            if self.brightness != 50 or self.contrast != 50:
                alpha = self.contrast / 50.0
                beta = (self.brightness - 50) * 2.0
                frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)

            _, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            return enc.tobytes()


driver = StandaloneSTS3215Driver()
camera_manager = StandaloneCameraManager()
connected_websockets: Set[tornado.websocket.WebSocketHandler] = set()
active_targets: Dict[str, float] = {}
is_recording = False
record_file = None
record_writer = None
sim_positions: Dict[str, float] = {}


class BaseApiHandler(tornado.web.RequestHandler):
    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Headers", "Content-Type, Authorization")
        self.set_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")

    def options(self, *args, **kwargs):
        self.set_status(204)
        self.finish()


class CameraDevicesHandler(BaseApiHandler):
    def get(self):
        devices = camera_manager.list_devices()
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps({"success": True, "devices": devices}))


class CameraConnectHandler(BaseApiHandler):
    def post(self):
        try:
            data = json.loads(self.request.body)
            dev_id = data.get("device_id", "0")
            cam_type = data.get("camera_type", "usb")
            width = int(data.get("width", 640))
            height = int(data.get("height", 480))
            fps = int(data.get("fps", 30))
            custom_url = data.get("custom_url")
            res = camera_manager.connect(dev_id, cam_type, width, height, fps, custom_url)
            self.set_header("Content-Type", "application/json")
            self.write(json.dumps(res))
        except Exception as e:
            self.write(json.dumps({"success": False, "message": str(e)}))


class CameraDisconnectHandler(BaseApiHandler):
    def post(self):
        camera_manager.disconnect()
        self.write(json.dumps({"success": True, "message": "Camera disconnected."}))


class CameraSettingsHandler(BaseApiHandler):
    def post(self):
        try:
            data = json.loads(self.request.body)
            if "brightness" in data: camera_manager.brightness = int(data["brightness"])
            if "contrast" in data: camera_manager.contrast = int(data["contrast"])
            if "stream_mode" in data: camera_manager.stream_mode = str(data["stream_mode"])
            self.write(json.dumps({"success": True}))
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
        self.set_header("Access-Control-Allow-Origin", "*")

        while self._is_active:
            frame_bytes = camera_manager.get_jpeg_frame(mode=mode)
            if frame_bytes:
                try:
                    self.write(b"--frame\r\n")
                    self.write(b"Content-Type: image/jpeg\r\n")
                    self.write(f"Content-Length: {len(frame_bytes)}\r\n\r\n".encode())
                    self.write(frame_bytes)
                    self.write(b"\r\n")
                    yield self.flush()
                except Exception:
                    break
            yield tornado.gen.sleep(0.04)


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
        (r"/api/camera/devices", CameraDevicesHandler),
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
    args = parser.parse_args()

    ports = list_available_serial_ports()
    target_port = args.port
    if not target_port and ports:
        target_port = ports[0]
    elif not target_port:
        target_port = "COM3" if sys.platform.startswith("win") else "/dev/ttyACM0"

    print("=================================================================")
    print("      SO-101 ROBOTICS STANDALONE HARDWARE BRIDGE (ZERO ROS 2)     ")
    print("=================================================================")
    print(f" Available ports : {ports if ports else 'None detected'}")
    print(f" Attempting port : {target_port} at {args.baud} baud")
    print(f" Web UI URL      : http://localhost:{args.web_port}")
    print("=================================================================\n")

    driver.connect(target_port, args.baud)

    home = load_home_position()
    for k, v in home.items():
        active_targets[k] = v
        sim_positions[k] = v

    app = make_app()
    app.listen(args.web_port)

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
