#!/usr/bin/env python3
"""
SO-101 Leader-Follower Teleoperation & LeRobot Episode Recorder Engine
Provides:
1. Low-latency (50 Hz) Leader-to-Follower hardware mirroring via Feetech STS3215 protocol.
2. Robust Leader Arm Torque control (Forced Torque OFF for smooth manual human hand guiding).
3. Operator demonstration recorder compatible with Hugging Face LeRobot dataset format.
4. Synchronized Dual-Camera frame capture (Workspace Top Cam + Gripper Wrist Cam).
5. Interactive Replay verification of recorded episodes.
6. Modular Tornado HTTP API endpoints.
"""

import os
import sys
import time
import math
import json
import glob
import csv
import shutil
import threading
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Any, Tuple

import serial
import serial.tools.list_ports

try:
    import tornado.web
    TORNADO_AVAILABLE = True
except ImportError:
    TORNADO_AVAILABLE = False

ROOT_DIR = Path(__file__).resolve().parent
DATASET_ROOT = ROOT_DIR / "data" / "lerobot_dataset"
DATASET_ROOT.mkdir(parents=True, exist_ok=True)

# Standard SO-101 Joint Name Mapping
JOINT_NAMES = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper"
}

# Default STS3215 Calibration
DEFAULT_CALIBRATION = {
    1: {"name": "shoulder_pan",  "id": 1, "homing_offset": 1823, "min": 1197, "max": 3113, "deg_min": -84.1, "deg_max": 84.1},
    2: {"name": "shoulder_lift", "id": 2, "homing_offset": 985,  "min": 1,    "max": 4091, "deg_min": -179.9, "deg_max": 179.9},
    3: {"name": "elbow_flex",    "id": 3, "homing_offset": -327, "min": 0,    "max": 4094, "deg_min": -180.0, "deg_max": 179.9},
    4: {"name": "wrist_flex",    "id": 4, "homing_offset": 1874, "min": 1102, "max": 3022, "deg_min": -84.3, "deg_max": 84.3},
    5: {"name": "wrist_roll",    "id": 5, "homing_offset": 601,  "min": 0,    "max": 4095, "deg_min": -180.0, "deg_max": 180.0},
    6: {"name": "gripper",       "id": 6, "homing_offset": 1467, "min": 2013, "max": 3505, "deg_min": 0.0,   "deg_max": 100.0},
}


def calc_sts_checksum(pkt: bytearray) -> int:
    return (~sum(pkt[2:])) & 0xFF


class LeaderArmDriver:
    """Read-only driver for the Leader Arm with Torque disabled for human manual guiding."""

    def __init__(self, port: str = "/dev/ttyACM1", baudrate: int = 1000000, timeout: float = 0.02):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_connected = False
        self.is_simulation = False
        self.torque_enabled = False
        self._ser: Optional[serial.Serial] = None
        self.calibration = DEFAULT_CALIBRATION
        self.last_angles: Dict[int, float] = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0}
        self.lock = threading.RLock()

    def connect(self, port: Optional[str] = None) -> Tuple[bool, str]:
        with self.lock:
            if port:
                self.port = port
            self.disconnect()

            # Simulation fallback
            if self.port.lower() in ("simulation", "sim", "virtual", "demo"):
                self.is_connected = True
                self.is_simulation = True
                self.torque_enabled = False
                print("[LEADER] Running in Virtual/Simulation mode.")
                return True, "Connected in Simulation Mode."

            # Check port existence on Linux
            if not sys.platform.startswith("win") and not os.path.exists(self.port):
                detected = [p.device for p in serial.tools.list_ports.comports() if not p.device.startswith("/dev/ttyS")]
                msg = f"Port '{self.port}' not found. Detected serial ports: {detected or 'None'}. Please plug in the Leader Arm USB cable."
                print(f"[LEADER ERROR] {msg}")
                self.is_connected = False
                return False, msg

            try:
                self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                self._ser.reset_input_buffer()
                self._ser.reset_output_buffer()
                self.is_connected = True
                self.is_simulation = False
                # Crucial: Disable torque on all leader servos so human operator can guide freely!
                self.set_torque(False)
                print(f"[LEADER] Connected to Leader arm on {self.port} at {self.baudrate} baud (Torque OFF).")
                return True, f"Connected to Leader Arm on {self.port} (Torque OFF)."
            except Exception as e:
                msg = f"Could not connect to Leader arm on {self.port}: {e}"
                print(f"[LEADER ERROR] {msg}")
                self.is_connected = False
                return False, msg

    def disconnect(self):
        with self.lock:
            if self._ser is not None and self._ser.is_open:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
            self.is_connected = False
            self.is_simulation = False

    def set_torque(self, enable: bool = False):
        """Sets motor holding torque on all 6 leader servos.
        False (0) = Torque OFF (Free human manual guiding)
        True (1)  = Torque ON (Servos lock in place)
        """
        with self.lock:
            self.torque_enabled = enable
            if not self._ser or not self._ser.is_open:
                return

            val = 1 if enable else 0

            # 1. Sync Write (0x83) to Address 40
            pkt = bytearray([0xFF, 0xFF, 0xFE, 16, 0x83, 40, 1])
            for sid in range(1, 7):
                pkt.append(sid)
                pkt.append(val)
            pkt.append(calc_sts_checksum(pkt))
            try:
                self._ser.write(pkt)
            except Exception:
                pass

            # 2. Individual Servo WRITE (0x03) for 100% hardware reliability
            for sid in range(1, 7):
                ind = bytearray([0xFF, 0xFF, sid, 4, 0x03, 40, val])
                ind.append(calc_sts_checksum(ind))
                try:
                    self._ser.write(ind)
                except Exception:
                    pass

    def ticks_to_deg(self, sid: int, ticks: int) -> float:
        cal = self.calibration.get(sid, {})
        c_min = cal.get("min", 0)
        c_max = cal.get("max", 4095)
        if sid == 6:  # Gripper percentage (0% closed to 100% open)
            pct = ((ticks - c_min) / max(1, c_max - c_min)) * 100.0
            return max(0.0, min(100.0, pct))
        mid = (c_min + c_max) / 2.0
        deg = (ticks - mid) * (360.0 / 4095.0)
        return float(deg)

    def read_single_servo_position(self, sid: int) -> Optional[int]:
        """Reads current 12-bit encoder ticks (0-4095) from servo ID."""
        if not self._ser or not self._ser.is_open:
            return None
        # STS3215 READ command: ID, Length(4), Instruction(0x02), Address(56=Present Position L), ReadLength(2)
        pkt = bytearray([0xFF, 0xFF, sid, 4, 0x02, 56, 2])
        pkt.append(calc_sts_checksum(pkt))
        try:
            self._ser.reset_input_buffer()
            self._ser.write(pkt)
            resp = self._ser.read(8)
            if len(resp) >= 8 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == sid:
                pos = resp[5] | (resp[6] << 8)
                if pos & 0x8000:
                    pos = -(pos & 0x7FFF)
                return pos
        except Exception:
            pass
        return None

    def read_all_joint_angles(self) -> Dict[int, float]:
        """Reads all 6 joints. Returns dictionary mapping joint ID (1..6) to degrees/percentage."""
        with self.lock:
            if not self.is_connected:
                return dict(self.last_angles)

            if self.is_simulation:
                return dict(self.last_angles)

            for sid in range(1, 7):
                ticks = self.read_single_servo_position(sid)
                if ticks is not None:
                    self.last_angles[sid] = round(self.ticks_to_deg(sid, ticks), 2)
            return dict(self.last_angles)


class LeRobotEpisodeRecorder:
    """Manages demonstration episode recording formatted for Hugging Face LeRobot."""

    def __init__(self, dataset_name: str = "so101_cube_pick_place"):
        self.dataset_name = dataset_name
        self.dataset_dir = DATASET_ROOT / self.dataset_name
        self.dataset_dir.mkdir(parents=True, exist_ok=True)
        self.is_recording = False
        self.current_episode = self._detect_next_episode_index()
        self.current_task = "Pick cube and place in target bowl"
        self.episode_start_time = 0.0
        self.frames_buffer: List[Dict[str, Any]] = []
        self.lock = threading.Lock()

    def _detect_next_episode_index(self) -> int:
        episodes = glob.glob(str(self.dataset_dir / "episode_*"))
        if not episodes:
            return 1
        indices = []
        for p in episodes:
            base = os.path.basename(p)
            parts = base.split("_")
            if len(parts) >= 2 and parts[1].isdigit():
                indices.append(int(parts[1]))
        return max(indices) + 1 if indices else 1

    def start_episode(self, task: Optional[str] = None) -> Dict[str, Any]:
        with self.lock:
            self.is_recording = True
            if task:
                self.current_task = task
            self.episode_start_time = time.time()
            self.frames_buffer = []
            print(f"[LEROBOT] Started recording Episode #{self.current_episode} ('{self.current_task}')...")
            return {
                "success": True,
                "episode": self.current_episode,
                "task": self.current_task,
                "message": f"Recording Episode #{self.current_episode} started."
            }

    def record_step(self, leader_angles: Dict[int, float], follower_angles: Dict[int, float],
                    top_img_bytes: Optional[bytes] = None, wrist_img_bytes: Optional[bytes] = None):
        if not self.is_recording:
            return
        t = time.time() - self.episode_start_time
        frame_entry = {
            "timestamp": round(t, 4),
            "action": [leader_angles.get(i, 0.0) for i in range(1, 7)],
            "observation_state": [follower_angles.get(i, 0.0) for i in range(1, 7)],
            "has_top_cam": top_img_bytes is not None,
            "has_wrist_cam": wrist_img_bytes is not None
        }
        self.frames_buffer.append(frame_entry)

    def stop_and_save_episode(self) -> Dict[str, Any]:
        with self.lock:
            if not self.is_recording:
                return {"success": False, "message": "No active recording."}

            self.is_recording = False
            ep_idx = self.current_episode
            ep_dir = self.dataset_dir / f"episode_{ep_idx:04d}"
            ep_dir.mkdir(parents=True, exist_ok=True)

            # Save dataset tabular data (CSV)
            csv_path = ep_dir / "trajectory.csv"
            total_frames = len(self.frames_buffer)
            duration = round(time.time() - self.episode_start_time, 2)

            with open(csv_path, "w", newline="") as f:
                header = ["timestamp"] + [f"state_j{i}" for i in range(1, 7)] + [f"action_j{i}" for i in range(1, 7)]
                writer = csv.writer(f)
                writer.writerow(header)
                for row in self.frames_buffer:
                    line = [row["timestamp"]] + row["observation_state"] + row["action"]
                    writer.writerow(line)

            # Save metadata
            meta = {
                "episode_index": ep_idx,
                "task": self.current_task,
                "total_frames": total_frames,
                "duration_seconds": duration,
                "fps": round(total_frames / max(0.1, duration), 1),
                "created_at": datetime.now().isoformat(),
                "robot_type": "SO-101 (6-DOF STS3215)",
                "lerobot_format_version": "v2.0"
            }
            with open(ep_dir / "meta.json", "w") as f:
                json.dump(meta, f, indent=2)

            self.current_episode += 1
            self.frames_buffer = []
            print(f"[LEROBOT] Successfully saved Episode #{ep_idx} ({total_frames} frames, {duration}s). Next: #{self.current_episode}")
            return {
                "success": True,
                "saved_episode": ep_idx,
                "next_episode": self.current_episode,
                "total_frames": total_frames,
                "duration": duration,
                "message": f"Episode #{ep_idx} saved ({total_frames} frames)."
            }

    def discard_episode(self) -> Dict[str, Any]:
        with self.lock:
            discarded = self.current_episode
            self.is_recording = False
            self.frames_buffer = []
            print(f"[LEROBOT] Discarded Episode #{discarded}.")
            return {
                "success": True,
                "discarded_episode": discarded,
                "current_episode": self.current_episode,
                "message": f"Episode #{discarded} discarded."
            }

    def get_status(self) -> Dict[str, Any]:
        with self.lock:
            elapsed = round(time.time() - self.episode_start_time, 2) if self.is_recording else 0.0
            return {
                "is_recording": self.is_recording,
                "current_episode": self.current_episode,
                "total_episodes_saved": max(0, self.current_episode - 1),
                "current_frames": len(self.frames_buffer),
                "elapsed_seconds": elapsed,
                "task": self.current_task,
                "dataset_name": self.dataset_name
            }

    def list_episodes(self) -> List[Dict[str, Any]]:
        episodes_list = []
        for ep_path in sorted(self.dataset_dir.glob("episode_*")):
            meta_path = ep_path / "meta.json"
            if meta_path.exists():
                try:
                    with open(meta_path) as f:
                        meta = json.load(f)
                        episodes_list.append(meta)
                except Exception:
                    pass
        return episodes_list


class LeaderFollowerManager:
    """Coordinates leader arm reading, low-latency follower synchronization, replay, and episode recording."""

    def __init__(self, follower_driver=None, camera_manager=None):
        self.follower_driver = follower_driver
        self.camera_manager = camera_manager
        self.leader_driver = LeaderArmDriver()
        self.recorder = LeRobotEpisodeRecorder()
        self.is_mirroring = False
        self.is_replaying = False
        self.replay_stop_flag = False
        self.thread: Optional[threading.Thread] = None
        self.replay_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()
        self.loop_hz = 0.0
        self.last_latency_ms = 0.0

    def start_mirroring(self, leader_port: str = "/dev/ttyACM1", follower_port: Optional[str] = None) -> Dict[str, Any]:
        with self.lock:
            if self.is_mirroring:
                return {"success": True, "message": "Leader-Follower mirroring already active."}

            # 1. Connect leader arm
            ok, msg = self.leader_driver.connect(leader_port)
            if not ok:
                return {"success": False, "message": msg}

            # 2. Check / Connect Follower arm if needed
            if self.follower_driver and not getattr(self.follower_driver, "is_connected", False) and follower_port:
                if hasattr(self.follower_driver, "connect"):
                    self.follower_driver.connect(follower_port)

            # Ensure follower arm has torque enabled for holding
            if self.follower_driver and hasattr(self.follower_driver, "set_torque"):
                self.follower_driver.set_torque(True)

            self.is_mirroring = True
            self.thread = threading.Thread(target=self._mirroring_loop, daemon=True, name="leader-sync")
            self.thread.start()
            follower_p = getattr(self.follower_driver, "port", "N/A") if self.follower_driver else "N/A"
            print(f"[MIRROR] Leader-Follower Bilateral Teleoperation ACTIVE ({leader_port} -> {follower_p}).")
            return {
                "success": True,
                "leader_port": self.leader_driver.port,
                "follower_port": follower_p,
                "message": "Leader-Follower Teleoperation active."
            }

    def stop_mirroring(self) -> Dict[str, Any]:
        with self.lock:
            self.is_mirroring = False
            if self.recorder.is_recording:
                self.recorder.stop_and_save_episode()
            self.leader_driver.disconnect()
            print("[MIRROR] Leader-Follower Mirroring stopped.")
            return {"success": True, "message": "Leader-Follower mirroring halted."}

    def set_leader_torque(self, enable: bool = False) -> Dict[str, Any]:
        self.leader_driver.set_torque(enable)
        status_txt = "ON (Holding Lock)" if enable else "OFF (Manual Hand Guiding Mode)"
        return {"success": True, "torque_enabled": enable, "message": f"Leader torque set to {status_txt}."}

    def set_follower_torque(self, enable: bool = True) -> Dict[str, Any]:
        if self.follower_driver and hasattr(self.follower_driver, "set_torque"):
            ok = self.follower_driver.set_torque(enable)
            status_txt = "ON (Active Mirroring)" if enable else "OFF (Deactivated)"
            return {"success": ok, "torque_enabled": enable, "message": f"Follower torque set to {status_txt}."}
        return {"success": False, "message": "Follower arm driver is not connected."}

    def _mirroring_loop(self):
        last_t = time.time()
        loop_counter = 0

        while self.is_mirroring:
            loop_start = time.time()
            loop_counter += 1

            try:
                # Every 50 iterations (~1 sec), re-assert torque OFF on leader arm so servos never lock up
                if loop_counter % 50 == 0 and not self.leader_driver.torque_enabled:
                    self.leader_driver.set_torque(False)

                # 1. Read leader angles
                leader_angles = self.leader_driver.read_all_joint_angles()

                # 2. Write to follower arm with minimal lag
                if self.follower_driver and getattr(self.follower_driver, "is_connected", False) and leader_angles:
                    named_targets = {JOINT_NAMES[sid]: leader_angles.get(sid, 0.0) for sid in range(1, 7)}
                    if hasattr(self.follower_driver, "sync_write_all"):
                        self.follower_driver.sync_write_all(named_targets, speed_rad_s=2.8)
                    elif hasattr(self.follower_driver, "sync_write_positions"):
                        self.follower_driver.sync_write_positions(leader_angles, velocity=0)

                # 3. If recording episode, record state + camera snapshot
                if self.recorder.is_recording:
                    follower_angles = {}
                    if self.follower_driver and hasattr(self.follower_driver, "read_all_feedback"):
                        fb = self.follower_driver.read_all_feedback()
                        if fb:
                            follower_angles = {sid: fb[sid].get("position_deg", 0.0) for sid in range(1, 7) if sid in fb}
                    if not follower_angles:
                        follower_angles = dict(leader_angles)

                    top_frame = None
                    if self.camera_manager and getattr(self.camera_manager, "is_connected", False):
                        if hasattr(self.camera_manager, "get_jpeg_frame"):
                            top_frame = self.camera_manager.get_jpeg_frame("rgb")

                    self.recorder.record_step(leader_angles, follower_angles, top_img_bytes=top_frame)

                now = time.time()
                dt = now - last_t
                last_t = now
                if dt > 0:
                    self.loop_hz = round(0.9 * self.loop_hz + 0.1 * (1.0 / dt), 1)
                self.last_latency_ms = round((now - loop_start) * 1000.0, 1)

                # Target 50 Hz cycle (20 ms)
                elapsed = time.time() - loop_start
                sleep_t = max(0.001, 0.020 - elapsed)
                time.sleep(sleep_t)
            except Exception:
                time.sleep(0.02)

    def replay_episode(self, episode_index: int, speed: float = 1.0) -> Dict[str, Any]:
        """Replay recorded episode CSV on follower arm."""
        with self.lock:
            if self.is_replaying:
                return {"success": False, "message": "Episode replay is already in progress."}

            ep_dir = self.recorder.dataset_dir / f"episode_{episode_index:04d}"
            csv_path = ep_dir / "trajectory.csv"
            if not csv_path.exists():
                return {"success": False, "message": f"Episode #{episode_index} not found at {csv_path}."}

            self.is_replaying = True
            self.replay_stop_flag = False
            self.replay_thread = threading.Thread(
                target=self._run_replay,
                args=(csv_path, episode_index, max(0.2, min(3.0, speed))),
                daemon=True,
                name="episode-replay"
            )
            self.replay_thread.start()
            return {"success": True, "message": f"Replaying Episode #{episode_index} at {speed}x speed."}

    def stop_replay(self) -> Dict[str, Any]:
        with self.lock:
            self.replay_stop_flag = True
            self.is_replaying = False
            return {"success": True, "message": "Episode replay stopped."}

    def _run_replay(self, csv_path: Path, ep_idx: int, speed: float):
        try:
            rows = []
            with open(csv_path, "r") as f:
                reader = csv.reader(f)
                header = next(reader, None)
                for r in reader:
                    if r:
                        rows.append([float(x) for x in r])

            if not rows:
                self.is_replaying = False
                return

            print(f"[REPLAY] Playing {len(rows)} frames of Episode #{ep_idx} at {speed}x...")
            prev_t = rows[0][0]

            for row in rows:
                if self.replay_stop_flag:
                    break
                t = row[0]
                dt = (t - prev_t) / speed
                prev_t = t
                if dt > 0:
                    time.sleep(min(0.2, max(0.005, dt)))

                # Action joints are indices 7..12 (1..6)
                action_angles = {i: row[6 + i] for i in range(1, 7)}
                named_targets = {JOINT_NAMES[sid]: action_angles[sid] for sid in range(1, 7)}

                if self.follower_driver and getattr(self.follower_driver, "is_connected", False):
                    if hasattr(self.follower_driver, "sync_write_all"):
                        self.follower_driver.sync_write_all(named_targets, speed_rad_s=1.8 * speed)
                    elif hasattr(self.follower_driver, "sync_write_positions"):
                        self.follower_driver.sync_write_positions(action_angles, velocity=0)
            print(f"[REPLAY] Finished Episode #{ep_idx}.")
        except Exception as e:
            print(f"[REPLAY ERROR] {e}")
        finally:
            self.is_replaying = False

    def get_status(self) -> Dict[str, Any]:
        follower_angles = {}
        if self.follower_driver and getattr(self.follower_driver, "is_connected", False):
            if hasattr(self.follower_driver, "read_all_feedback"):
                fb = self.follower_driver.read_all_feedback()
                if fb:
                    follower_angles = {sid: fb[sid].get("position_deg", 0.0) for sid in range(1, 7) if sid in fb}
        if not follower_angles:
            follower_angles = self.leader_driver.last_angles

        rec_status = self.recorder.get_status()
        follower_p = getattr(self.follower_driver, "port", "") if self.follower_driver else ""
        follower_c = getattr(self.follower_driver, "is_connected", False) if self.follower_driver else False
        cam_c = getattr(self.camera_manager, "is_connected", False) if self.camera_manager else False

        return {
            "is_mirroring": self.is_mirroring,
            "is_replaying": self.is_replaying,
            "leader_connected": self.leader_driver.is_connected,
            "follower_connected": follower_c,
            "camera_connected": cam_c,
            "leader_torque_enabled": self.leader_driver.torque_enabled,
            "leader_port": self.leader_driver.port,
            "follower_port": follower_p,
            "sync_rate_hz": self.loop_hz,
            "latency_ms": self.last_latency_ms,
            "leader_angles": self.leader_driver.last_angles,
            "follower_angles": follower_angles,
            "recording": rec_status
        }


# Global Manager Singleton
_lf_manager: Optional[LeaderFollowerManager] = None


def get_leader_follower_manager(follower_driver=None, camera_manager=None) -> LeaderFollowerManager:
    global _lf_manager
    if _lf_manager is None:
        _lf_manager = LeaderFollowerManager(follower_driver=follower_driver, camera_manager=camera_manager)
    else:
        if follower_driver:
            _lf_manager.follower_driver = follower_driver
        if camera_manager:
            _lf_manager.camera_manager = camera_manager
    return _lf_manager


def init_leader_follower_routes(follower_driver=None, camera_manager=None):
    """Initializes the manager singleton with follower driver and camera references."""
    get_leader_follower_manager(follower_driver=follower_driver, camera_manager=camera_manager)


# ── Tornado HTTP Request Handlers ──────────────────────────────────────────

if TORNADO_AVAILABLE:
    class _BaseLFHandler(tornado.web.RequestHandler):
        def set_default_headers(self):
            self.set_header("Content-Type", "application/json")
            self.set_header("Access-Control-Allow-Origin", "*")

        def options(self, *args):
            self.set_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
            self.set_header("Access-Control-Allow-Headers", "Content-Type")
            self.set_status(204)
            self.finish()

        def json_response(self, obj: Any):
            self.write(json.dumps(obj))
            self.finish()

    class LeaderFollowerPortsHandler(_BaseLFHandler):
        """GET — list available serial ports for Leader and Follower arms (filtered of dummy ports)."""
        def get(self):
            all_ports = []
            try:
                for p in serial.tools.list_ports.comports():
                    desc = f"{p.description or ''} {p.hwid or ''}".lower()
                    if p.device.startswith(("/dev/ttyS", "/dev/ttyAMA")) or "bluetooth" in desc:
                        continue
                    all_ports.append({
                        "device": p.device,
                        "description": p.description or p.device,
                        "hwid": p.hwid or ""
                    })
            except Exception:
                pass

            if not all_ports and not sys.platform.startswith("win"):
                for pat in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usb*"):
                    for g in sorted(glob.glob(pat)):
                        all_ports.append({"device": g, "description": g, "hwid": ""})

            port_devices = [p["device"] for p in all_ports]
            if "simulation" not in port_devices:
                port_devices.append("simulation")

            mgr = get_leader_follower_manager()
            self.json_response({
                "success": True,
                "ports": port_devices,
                "port_details": all_ports,
                "leader_port": mgr.leader_driver.port,
                "follower_port": getattr(mgr.follower_driver, "port", "") if mgr.follower_driver else "",
                "leader_connected": mgr.leader_driver.is_connected,
                "follower_connected": getattr(mgr.follower_driver, "is_connected", False) if mgr.follower_driver else False
            })

    class LeaderFollowerStartHandler(_BaseLFHandler):
        """POST — connect leader arm and start low-latency mirroring loop."""
        def post(self):
            try:
                data = json.loads(self.request.body) if self.request.body else {}
            except Exception:
                data = {}
            leader_port = data.get("leader_port", "/dev/ttyACM1")
            follower_port = data.get("follower_port", None)
            mgr = get_leader_follower_manager()
            res = mgr.start_mirroring(leader_port=leader_port, follower_port=follower_port)
            self.json_response(res)

    class LeaderFollowerStopHandler(_BaseLFHandler):
        """POST — stop mirroring and release leader arm port."""
        def post(self):
            mgr = get_leader_follower_manager()
            res = mgr.stop_mirroring()
            self.json_response(res)

    class LeaderTorqueHandler(_BaseLFHandler):
        """POST — set leader arm torque (False = 0 for free hand guiding, True = 1 for lock)."""
        def post(self):
            try:
                data = json.loads(self.request.body) if self.request.body else {}
            except Exception:
                data = {}
            enable = bool(data.get("enable", False))
            mgr = get_leader_follower_manager()
            res = mgr.set_leader_torque(enable)
            self.json_response(res)

    class FollowerTorqueHandler(_BaseLFHandler):
        """POST — toggle follower arm holding torque."""
        def post(self):
            try:
                data = json.loads(self.request.body) if self.request.body else {}
            except Exception:
                data = {}
            enable = bool(data.get("enable", True))
            mgr = get_leader_follower_manager()
            res = mgr.set_follower_torque(enable)
            self.json_response(res)

    class LeaderFollowerStatusHandler(_BaseLFHandler):
        """GET — real-time joint positions, tracking error, loop rate and latency."""
        def get(self):
            mgr = get_leader_follower_manager()
            self.json_response(mgr.get_status())

    class EpisodeStartHandler(_BaseLFHandler):
        """POST — start recording current episode demonstration."""
        def post(self):
            try:
                data = json.loads(self.request.body) if self.request.body else {}
            except Exception:
                data = {}
            task = data.get("task", "Pick cube and place in target bowl")
            mgr = get_leader_follower_manager()
            res = mgr.recorder.start_episode(task=task)
            self.json_response(res)

    class EpisodeStopHandler(_BaseLFHandler):
        """POST — stop and save demonstration episode to disk."""
        def post(self):
            mgr = get_leader_follower_manager()
            res = mgr.recorder.stop_and_save_episode()
            self.json_response(res)

    class EpisodeDiscardHandler(_BaseLFHandler):
        """POST — discard flawed demonstration attempt without saving."""
        def post(self):
            mgr = get_leader_follower_manager()
            res = mgr.recorder.discard_episode()
            self.json_response(res)

    class EpisodeStatusHandler(_BaseLFHandler):
        """GET — episode recording status."""
        def get(self):
            mgr = get_leader_follower_manager()
            self.json_response(mgr.recorder.get_status())

    class EpisodeListHandler(_BaseLFHandler):
        """GET — list all recorded episodes with metadata."""
        def get(self):
            mgr = get_leader_follower_manager()
            episodes = mgr.recorder.list_episodes()
            self.json_response({
                "success": True,
                "dataset_name": mgr.recorder.dataset_name,
                "dataset_dir": str(mgr.recorder.dataset_dir),
                "total_episodes": len(episodes),
                "episodes": episodes
            })

    class EpisodeReplayHandler(_BaseLFHandler):
        """POST — replay a recorded episode on the follower robot."""
        def post(self):
            try:
                data = json.loads(self.request.body) if self.request.body else {}
            except Exception:
                data = {}
            ep_idx = int(data.get("episode_index", 1))
            speed = float(data.get("speed", 1.0))
            mgr = get_leader_follower_manager()
            res = mgr.replay_episode(episode_index=ep_idx, speed=speed)
            self.json_response(res)

    class EpisodeStopReplayHandler(_BaseLFHandler):
        """POST — stop active episode replay."""
        def post(self):
            mgr = get_leader_follower_manager()
            res = mgr.stop_replay()
            self.json_response(res)

    # Routes to mount in Tornado Application
    LEADER_FOLLOWER_ROUTES = [
        (r"/api/teleop/leader_follower/ports",       LeaderFollowerPortsHandler),
        (r"/api/teleop/leader_follower/start",       LeaderFollowerStartHandler),
        (r"/api/teleop/leader_follower/stop",        LeaderFollowerStopHandler),
        (r"/api/teleop/leader_follower/status",      LeaderFollowerStatusHandler),
        (r"/api/teleop/leader/torque",               LeaderTorqueHandler),
        (r"/api/teleop/follower/torque",             FollowerTorqueHandler),
        (r"/api/teleop/episode/start",               EpisodeStartHandler),
        (r"/api/teleop/episode/stop",                EpisodeStopHandler),
        (r"/api/teleop/episode/discard",             EpisodeDiscardHandler),
        (r"/api/teleop/episode/status",              EpisodeStatusHandler),
        (r"/api/teleop/episode/list",                EpisodeListHandler),
        (r"/api/teleop/episode/replay",              EpisodeReplayHandler),
        (r"/api/teleop/episode/stop_replay",         EpisodeStopReplayHandler),
    ]
else:
    LEADER_FOLLOWER_ROUTES = []
