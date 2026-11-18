#!/usr/bin/env python3
"""
SO-101 Trajectory Recorder, Playback & Dynamics Testing Module
Supports multi-loop trajectory playback, CSV dataset recording, and hardware characterization.
"""

import os
import csv
import time
import math
import threading
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Any, Callable

ROOT_DIR = Path(__file__).resolve().parent
DATA_DIR = ROOT_DIR / "data" / "raw"
DATA_DIR.mkdir(parents=True, exist_ok=True)


class TrajectoryRecorder:
    """Manages CSV dataset recording, looping playback, and hardware testing."""

    def __init__(self, driver=None, teleop_controller=None):
        self.driver = driver
        self.teleop = teleop_controller

        self.is_recording = False
        self.record_file = None
        self.record_writer = None
        self.record_lock = threading.Lock()

        self.is_playing = False
        self.playback_thread: Optional[threading.Thread] = None
        self.playback_stop_evt = threading.Event()
        self.current_loop = 0
        self.total_loops = 1

    def start_recording(self, filename: Optional[str] = None) -> Dict[str, Any]:
        """Start streaming joint states into a CSV trajectory file."""
        with self.record_lock:
            if self.is_recording:
                return {"success": False, "message": "Already recording."}
            if not filename:
                filename = f"trajectory_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            filepath = DATA_DIR / filename
            try:
                self.record_file = open(filepath, "w", newline="")
                fieldnames = [
                    "timestamp",
                    "shoulder_pan_deg", "shoulder_lift_deg", "elbow_flex_deg",
                    "wrist_flex_deg", "wrist_roll_deg", "gripper_pct",
                    "shoulder_pan_rad", "shoulder_lift_rad", "elbow_flex_rad",
                    "wrist_flex_rad", "wrist_roll_rad", "gripper_val"
                ]
                self.record_writer = csv.DictWriter(self.record_file, fieldnames=fieldnames)
                self.record_writer.writeheader()
                self.is_recording = True
                print(f"[REC] Started recording trajectory to {filepath}")
                return {"success": True, "filename": filename, "path": str(filepath)}
            except Exception as e:
                self.is_recording = False
                return {"success": False, "message": str(e)}

    def record_step(self, joint_states: Dict[str, Any]):
        """Write a single timestamped frame to the active CSV file."""
        if not self.is_recording or not self.record_writer:
            return
        with self.record_lock:
            try:
                row = {
                    "timestamp": round(time.time(), 4),
                    "shoulder_pan_deg": round(joint_states.get("shoulder_pan", {}).get("position_deg", 0.0), 2),
                    "shoulder_lift_deg": round(joint_states.get("shoulder_lift", {}).get("position_deg", 0.0), 2),
                    "elbow_flex_deg": round(joint_states.get("elbow_flex", {}).get("position_deg", 0.0), 2),
                    "wrist_flex_deg": round(joint_states.get("wrist_flex", {}).get("position_deg", 0.0), 2),
                    "wrist_roll_deg": round(joint_states.get("wrist_roll", {}).get("position_deg", 0.0), 2),
                    "gripper_pct": round(joint_states.get("gripper", {}).get("position_deg", 0.0), 2),
                    "shoulder_pan_rad": round(joint_states.get("shoulder_pan", {}).get("position_rad", 0.0), 4),
                    "shoulder_lift_rad": round(joint_states.get("shoulder_lift", {}).get("position_rad", 0.0), 4),
                    "elbow_flex_rad": round(joint_states.get("elbow_flex", {}).get("position_rad", 0.0), 4),
                    "wrist_flex_rad": round(joint_states.get("wrist_flex", {}).get("position_rad", 0.0), 4),
                    "wrist_roll_rad": round(joint_states.get("wrist_roll", {}).get("position_rad", 0.0), 4),
                    "gripper_val": round(joint_states.get("gripper", {}).get("position_rad", 0.0), 4),
                }
                self.record_writer.writerow(row)
                self.record_file.flush()
            except Exception:
                pass

    def stop_recording(self) -> Dict[str, Any]:
        """Stop recording and flush the CSV file to disk."""
        with self.record_lock:
            if not self.is_recording:
                return {"success": False, "message": "Not recording."}
            self.is_recording = False
            if self.record_file:
                try:
                    self.record_file.close()
                except Exception:
                    pass
                self.record_file = None
            self.record_writer = None
            print("[REC] Recording stopped successfully.")
            return {"success": True, "message": "Recording saved."}

    def start_playback(self, filename: str, loop_count: int = 1,
                       on_progress: Optional[Callable[[int, int, float], None]] = None) -> Dict[str, Any]:
        """Start replaying a recorded trajectory with configurable loop count."""
        if self.is_playing:
            return {"success": False, "message": "Playback is already running."}

        if self.teleop:
            allowed, reason = self.teleop.can_execute_movement()
            if not allowed:
                return {"success": False, "message": reason}

        filepath = DATA_DIR / filename
        if not filepath.is_file():
            # Search in DATA_DIR
            cands = list(DATA_DIR.glob(filename))
            if cands:
                filepath = cands[0]
            else:
                return {"success": False, "message": f"File '{filename}' not found."}

        # Parse CSV rows
        try:
            with open(filepath, "r") as f:
                reader = csv.DictReader(f)
                rows = list(reader)
        except Exception as e:
            return {"success": False, "message": f"Failed to read CSV: {e}"}

        if not rows:
            return {"success": False, "message": "Trajectory file is empty."}

        self.total_loops = max(1, int(loop_count))
        self.current_loop = 0
        self.playback_stop_evt.clear()
        self.is_playing = True

        def playback_worker():
            print(f"[PLAY] Starting playback of {filename} ({len(rows)} frames, {self.total_loops} loop(s)).")
            for loop_idx in range(self.total_loops):
                if self.playback_stop_evt.is_set():
                    break
                self.current_loop = loop_idx + 1
                t0_real = time.time()
                t0_file = float(rows[0].get("timestamp", 0))

                for i, row in enumerate(rows):
                    if self.playback_stop_evt.is_set():
                        break

                    # Check safety during playback
                    if self.teleop:
                        allowed, _ = self.teleop.can_execute_movement()
                        if not allowed:
                            print("[PLAY] Playback aborted due to safety interlock.")
                            break

                    file_time = float(row.get("timestamp", 0)) - t0_file
                    elapsed = time.time() - t0_real
                    sleep_dur = file_time - elapsed
                    if sleep_dur > 0:
                        time.sleep(sleep_dur)

                    targets = {
                        "shoulder_pan": float(row.get("shoulder_pan_deg", 0)),
                        "shoulder_lift": float(row.get("shoulder_lift_deg", 0)),
                        "elbow_flex": float(row.get("elbow_flex_deg", 0)),
                        "wrist_flex": float(row.get("wrist_flex_deg", 0)),
                        "wrist_roll": float(row.get("wrist_roll_deg", 0)),
                        "gripper": float(row.get("gripper_pct", 0)),
                    }
                    if self.driver and self.driver.is_connected:
                        self.driver.sync_write_positions(targets, velocity=2.0)

                    if on_progress:
                        pct = round(((i + 1) / len(rows)) * 100, 1)
                        on_progress(self.current_loop, self.total_loops, pct)

            self.is_playing = False
            print("[PLAY] Playback completed.")

        self.playback_thread = threading.Thread(target=playback_worker, daemon=True, name="traj-playback")
        self.playback_thread.start()
        return {
            "success": True,
            "message": f"Playback started ({self.total_loops} loops)",
            "loops": self.total_loops
        }

    def stop_playback(self) -> Dict[str, Any]:
        """Halt trajectory playback immediately."""
        self.playback_stop_evt.set()
        self.is_playing = False
        if self.playback_thread and self.playback_thread.is_alive():
            self.playback_thread.join(timeout=0.5)
        self.playback_thread = None
        print("[PLAY] Playback stopped.")
        return {"success": True, "message": "Playback stopped."}
