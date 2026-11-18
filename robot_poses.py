#!/usr/bin/env python3
"""
SO-101 Robot Poses Module
Manages Home, Zero, and Rest positions with file persistence and degree conversion.
"""

from pathlib import Path
from datetime import datetime
from typing import Dict, Any

ROOT_DIR = Path(__file__).resolve().parent

DEFAULT_HOME_POSITION = {
    "shoulder_pan": 0.88,
    "shoulder_lift": -104.62,
    "elbow_flex": 100.57,
    "wrist_flex": 59.96,
    "wrist_roll": 1.71,
    "gripper": 3.22
}

DEFAULT_ZERO_POSITION = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0
}

DEFAULT_REST_POSITION = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -90.0,
    "elbow_flex": 90.0,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.0
}


def load_home_position(filepath: Path = ROOT_DIR / "home_position.txt") -> Dict[str, float]:
    """Load the calibrated Home Position from disk or return default."""
    pos = dict(DEFAULT_HOME_POSITION)
    if filepath.is_file():
        try:
            for line in filepath.read_text().splitlines():
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if ":" in line:
                    k, v = line.split(":", 1)
                    k = k.strip().replace(".pos", "")
                    pos[k] = float(v.strip())
        except Exception as e:
            print(f"[WARN] Failed to load home position: {e}")
    return pos


def save_home_position(positions: Dict[str, float], filepath: Path = ROOT_DIR / "home_position.txt") -> bool:
    """Save target joint angles as the persistent Home Position."""
    try:
        lines = [
            "# SO-101 Home Position",
            f"# Saved on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            ""
        ]
        for k, v in positions.items():
            lines.append(f"{k}.pos: {float(v):.2f}")
        filepath.write_text("\n".join(lines) + "\n")
        print(f"[OK] Saved Home Position to {filepath}")
        return True
    except Exception as e:
        print(f"[ERROR] Could not save home position: {e}")
        return False


def load_zero_position(filepath: Path = ROOT_DIR / "zero_position.txt") -> Dict[str, float]:
    """Load the Zero Position from disk or return default zeros."""
    pos = dict(DEFAULT_ZERO_POSITION)
    if filepath.is_file():
        try:
            for line in filepath.read_text().splitlines():
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if ":" in line:
                    k, v = line.split(":", 1)
                    k = k.strip().replace(".pos", "")
                    pos[k] = float(v.strip())
        except Exception as e:
            print(f"[WARN] Failed to load zero position: {e}")
    return pos


def save_zero_position(positions: Dict[str, float], filepath: Path = ROOT_DIR / "zero_position.txt") -> bool:
    """Save target joint angles as the persistent Zero Position."""
    try:
        lines = [
            "# SO-101 Zero Position",
            f"# Saved on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            ""
        ]
        for k, v in positions.items():
            lines.append(f"{k}.pos: {float(v):.2f}")
        filepath.write_text("\n".join(lines) + "\n")
        print(f"[OK] Saved Zero Position to {filepath}")
        return True
    except Exception as e:
        print(f"[ERROR] Could not save zero position: {e}")
        return False
