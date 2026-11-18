#!/usr/bin/env python3
"""
SO-101 Robot Calibration Module
Handles STS3215 raw encoder ticks, joint limits, and persistent JSON calibration.
"""

import json
from pathlib import Path
from typing import Dict, Any

ROOT_DIR = Path(__file__).resolve().parent

DEFAULT_CALIBRATION = {
    1: {"name": "shoulder_pan",  "id": 1, "homing_offset": 1823, "min": 1197, "max": 3113, "rad_min": -1.4678, "rad_max": 1.4678, "deg_min": -84.1, "deg_max": 84.1},
    2: {"name": "shoulder_lift", "id": 2, "homing_offset": 985,  "min": 1,    "max": 4091, "rad_min": -3.1398, "rad_max": 3.1398, "deg_min": -179.9, "deg_max": 179.9},
    3: {"name": "elbow_flex",    "id": 3, "homing_offset": -327, "min": 0,    "max": 4094, "rad_min": -3.1416, "rad_max": 3.1398, "deg_min": -180.0, "deg_max": 179.9},
    4: {"name": "wrist_flex",    "id": 4, "homing_offset": 1874, "min": 1102, "max": 3022, "rad_min": -1.4713, "rad_max": 1.4713, "deg_min": -84.3, "deg_max": 84.3},
    5: {"name": "wrist_roll",    "id": 5, "homing_offset": 601,  "min": 0,    "max": 4095, "rad_min": -3.1416, "rad_max": 3.1416, "deg_min": -180.0, "deg_max": 180.0},
    6: {"name": "gripper",       "id": 6, "homing_offset": 1467, "min": 2013, "max": 3505, "rad_min": 0.0,    "rad_max": 1.0,    "deg_min": 0.0,   "deg_max": 100.0},
}

STS_ENCODER_RESOLUTION = 4096


def load_calibration(filepath: Path = ROOT_DIR / "so101_calibration.json") -> Dict[int, Dict[str, Any]]:
    """Load calibration mapping from disk or fallback to DEFAULT_CALIBRATION."""
    calib = {k: dict(v) for k, v in DEFAULT_CALIBRATION.items()}
    if filepath.is_file():
        try:
            data = json.loads(filepath.read_text())
            name_to_sid = {v["name"]: k for k, v in DEFAULT_CALIBRATION.items()}
            for item in data.get("joints", []):
                sid = item.get("id") or name_to_sid.get(item.get("name"))
                if sid and sid in calib:
                    c = dict(calib[sid])
                    c["homing_offset"] = item.get("homing_offset", c["homing_offset"])
                    c["min"] = item.get("range_min", c["min"])
                    c["max"] = item.get("range_max", c["max"])
                    calib[sid] = c
            print(f"[OK] Loaded calibration from {filepath}")
        except Exception as e:
            print(f"[WARN] Failed to load calibration file: {e}")
    return calib


def save_calibration(calib: Dict[int, Dict[str, Any]], filepath: Path = ROOT_DIR / "so101_calibration.json") -> bool:
    """Save calibration dictionary to JSON file."""
    try:
        data = {
            "version": "1.0",
            "joints": [
                {
                    "id": sid,
                    "name": v.get("name"),
                    "homing_offset": v.get("homing_offset"),
                    "range_min": v.get("min"),
                    "range_max": v.get("max")
                }
                for sid, v in sorted(calib.items())
            ]
        }
        filepath.write_text(json.dumps(data, indent=2))
        print(f"[OK] Saved calibration to {filepath}")
        return True
    except Exception as e:
        print(f"[ERROR] Could not save calibration: {e}")
        return False
