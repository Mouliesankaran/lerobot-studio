#!/usr/bin/env python3
"""
SO-101 Real Robot Home & Zero Position Calibrator
Connects directly to STS3215 bus via rustypot using user calibration JSON.

Features:
1. Auto-detects active serial port (/dev/ttyACM*, /dev/ttyUSB*).
2. Disables motor torque so you can freely position the arm by hand to your desired home pose.
3. Averages 10 stable encoder samples using exact calibration mapping.
4. Saves calibrated values to home_position.txt.
5. Also calculates and updates zero_position.txt.
"""

import os
import sys
import time
import json
import glob
from pathlib import Path

CURR_DIR = Path(__file__).resolve().parent
HOME_FILE = CURR_DIR / "home_position.txt"
ZERO_FILE = CURR_DIR / "zero_position.txt"
CALIB_FILE = CURR_DIR / "so101_calibration.json"

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]


def find_serial_port() -> str:
    if "ROBOT_PORT" in os.environ and os.path.exists(os.environ["ROBOT_PORT"]):
        return os.environ["ROBOT_PORT"]
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if ports:
        return ports[0]
    return "/dev/ttyACM0"


def load_calibration():
    if CALIB_FILE.is_file():
        with open(CALIB_FILE) as f:
            return json.load(f)
    # Default fallback
    return {
        "shoulder_pan":  {"id": 1, "range_min": 1197, "range_max": 3113},
        "shoulder_lift": {"id": 2, "range_min": 1,    "range_max": 4091},
        "elbow_flex":    {"id": 3, "range_min": 0,    "range_max": 4094},
        "wrist_flex":    {"id": 4, "range_min": 1102, "range_max": 3022},
        "wrist_roll":    {"id": 5, "range_min": 0,    "range_max": 4095},
        "gripper":       {"id": 6, "range_min": 2013, "range_max": 3505}
    }


def ticks_to_value(name: str, ticks: int, calib: dict) -> float:
    c = calib.get(name, {})
    rmin = c.get("range_min", 0)
    rmax = c.get("range_max", 4095)
    if name == "gripper":
        val = ((ticks - rmin) / max(1, rmax - rmin)) * 100.0
        return round(max(0.0, min(100.0, val)), 2)
    else:
        mid = (rmin + rmax) / 2.0
        val = (ticks - mid) * 360.0 / 4095.0
        return round(val, 2)


def value_to_ticks(name: str, val: float, calib: dict) -> int:
    c = calib.get(name, {})
    rmin = c.get("range_min", 0)
    rmax = c.get("range_max", 4095)
    if name == "gripper":
        ticks = int((val / 100.0) * (rmax - rmin) + rmin)
    else:
        mid = (rmin + rmax) / 2.0
        ticks = int((val * 4095.0 / 360.0) + mid)
    return max(rmin, min(rmax, ticks))


def record_home():
    print("=" * 64)
    print("       SO-101 REAL ROBOT HOME POSITION CALIBRATOR")
    print("=" * 64)

    port = find_serial_port()
    calib = load_calibration()
    print(f"[*] Detected serial port: {port}")

    try:
        import rustypot
    except ImportError:
        sys.exit("[ERROR] rustypot package is required. Install via 'pip install rustypot'.")

    _, defn = rustypot.find_model("sts3215")
    defs = {c["id"]: defn for c in calib.values()}

    try:
        bus = rustypot.Bus(port, 1000000, 0.05, defs)
    except Exception as e:
        sys.exit(f"[ERROR] Could not open port {port}: {e}")

    print("[OK] Connected to hardware!")
    print("Disabling motor torque so you can move the arm freely by hand...")
    for c in calib.values():
        sid = c["id"]
        try:
            bus.write_register(sid, "torque_enable", 0)
        except Exception:
            pass

    print("[OK] Arm is limp (torque is OFF).\n")
    print("👉 Physically position the robot arm at your desired HOME pose.")
    try:
        input("Press [Enter] when the arm is in position to record home (or Ctrl+C to cancel)...")
    except (KeyboardInterrupt, EOFError):
        print("\n[!] Aborted by user.")
        bus.close()
        return

    print("\nReading stable joint positions (10 samples)...")
    samples = {name: [] for name in JOINTS}
    for _ in range(10):
        for name in JOINTS:
            sid = calib[name]["id"]
            try:
                ticks = bus.read_register(sid, "present_position")
                val = ticks_to_value(name, ticks, calib)
                samples[name].append(val)
            except Exception:
                pass
        time.sleep(0.05)

    mean_pos = {}
    spread = {}
    for name in JOINTS:
        vals = samples[name]
        if vals:
            mean_pos[name] = round(sum(vals) / len(vals), 2)
            spread[name] = round(max(vals) - min(vals), 2)
        else:
            mean_pos[name] = 0.0
            spread[name] = 0.0

    print("\nRecorded Home Values:")
    print("-" * 50)
    lines = [
        "# SO-101 Real Follower Arm Calibrated Home Position",
        f"# Recorded on: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "# Format: joint_name.pos: angle_in_degrees (or 0-100 for gripper)",
        "",
    ]
    for name in JOINTS:
        val = mean_pos[name]
        unit = "%" if name == "gripper" else "°"
        print(f"  {name:<15}.pos: {val:7.2f} {unit}  (spread: {spread[name]:.2f})")
        lines.append(f"{name}.pos: {val:.2f}")

    lines.append("")
    HOME_FILE.write_text("\n".join(lines))
    print("-" * 50)
    print(f"[OK] New home position saved to: {HOME_FILE.resolve()}")

    # Also update canonical zero position (all joints at neutral 0.0°)
    zero_lines = [
        "# SO-101 canonical neutral upright pose",
        f"# Generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "# Format: joint_name.pos: angle_in_degrees",
        "",
        "shoulder_pan.pos: 0.00",
        "shoulder_lift.pos: 0.00",
        "elbow_flex.pos: 0.00",
        "wrist_flex.pos: 0.00",
        "wrist_roll.pos: 0.00",
        "gripper.pos: 0.00",
        ""
    ]
    ZERO_FILE.write_text("\n".join(zero_lines))
    print(f"[OK] Neutral zero position saved to: {ZERO_FILE.resolve()}")

    try:
        ans = input("\nTest holding this new home pose with torque ON? [y/N]: ").strip().lower()
        if ans in ("y", "yes"):
            print("[*] Enabling torque and holding recorded position...")
            for name in JOINTS:
                sid = calib[name]["id"]
                target_ticks = value_to_ticks(name, mean_pos[name], calib)
                try:
                    bus.write_register(sid, "goal_position", target_ticks)
                    bus.write_register(sid, "torque_enable", 1)
                except Exception:
                    pass
            print("[OK] Arm is holding new home pose.")
            input("Press [Enter] to turn torque back OFF and exit...")
    except Exception:
        pass
    finally:
        print("\nRelaxing arm motors and disconnecting...")
        for c in calib.values():
            try:
                bus.write_register(c["id"], "torque_enable", 0)
            except Exception:
                pass
        bus.close()

    print("[OK] Home calibration complete! Arm is safely limp.\n")


if __name__ == "__main__":
    record_home()
