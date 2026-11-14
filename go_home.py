#!/usr/bin/env python3
"""Smoothly glides the SO-101 physical robot arm to its calibrated home position.

Usage:
    python3 go_home.py
    python3 go_home.py --duration 2.0   # Slew over 2 seconds
    python3 go_home.py --limp           # Disable torque immediately upon arrival
"""

import argparse
import os
import sys
import time
from pathlib import Path

# Clean ROS Humble path pollution from sys.path
sys.path = [p for p in sys.path if not p.startswith("/opt/ros")]

import numpy as np

CURR_DIR = Path(__file__).resolve().parent
HOME_FILE = CURR_DIR / "home_position.txt"
PORT = os.environ.get("ROBOT_PORT", "/dev/ttyACM0")
DEFAULT_ROBOT_ID = "my_awesome_follower_arm"

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ALL_JOINTS = ARM_JOINTS + ["gripper"]


def get_calibrated_robot_id():
    """Auto-detects registered follower robot ID."""
    if "ROBOT_ID" in os.environ:
        return os.environ["ROBOT_ID"]
    calib_dir = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "robots" / "so_follower"
    if calib_dir.is_dir():
        cands = list(calib_dir.glob("*.json"))
        if cands:
            return cands[0].stem
    if (CURR_DIR / "checkpoints" / "my_awesome_follower_arm.json").is_file():
        return "my_awesome_follower_arm"
    return DEFAULT_ROBOT_ID


def load_home_target():
    """Loads calibrated target angles from home_position.txt."""
    if not HOME_FILE.is_file():
        sys.exit(f"[ERROR] Home file not found: {HOME_FILE.resolve()}\nPlease run record_home.py first.")

    target = {}
    lines = HOME_FILE.read_text().splitlines()
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        if ":" in line:
            k, v = line.split(":", 1)
            k = k.strip()
            val = float(v.strip())
            if not k.endswith(".pos"):
                k = f"{k}.pos"
            target[k] = val

    for j in ALL_JOINTS:
        k = f"{j}.pos"
        if k not in target:
            sys.exit(f"[ERROR] Missing joint key '{k}' in {HOME_FILE.name}")
    return target


def connect_robot(port=PORT, robot_id=None):
    """Safely connects to the physical robot follower."""
    if robot_id is None:
        robot_id = get_calibrated_robot_id()
    print(f"[*] Connecting to physical SO-101 arm on {port} (id='{robot_id}')...")

    robot = None
    for cfg in [
        {"port": port, "id": robot_id, "use_degrees": True, "max_relative_target": 15.0},
        {"port": port, "id": robot_id, "use_degrees": True},
    ]:
        try:
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
            robot = SO101Follower(SO101FollowerConfig(**cfg))
            robot.connect(calibrate=False)
            break
        except Exception:
            try:
                from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
                robot = SO101Follower(SO101FollowerConfig(**cfg))
                robot.connect(calibrate=False)
                break
            except Exception:
                pass

    if robot is not None:
        # Ensure torque is enabled on all servos
        try:
            if hasattr(robot, "bus") and hasattr(robot.bus, "write"):
                for m_id in range(1, 7):
                    try: robot.bus.write("Torque_Enable", 1, m_id)
                    except Exception: pass
        except Exception:
            pass
        print(f"[OK] Connected to hardware successfully!\n")
        return robot
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=PORT, help=f"Serial port (default: {PORT})")
    ap.add_argument("--duration", type=float, default=2.0, help="Slew duration in seconds (default: 2.0s)")
    ap.add_argument("--limp", action="store_true", help="Turn OFF torque immediately after reaching home")
    args = ap.parse_args()

    print("=" * 72)
    print("          SO-101 PHYSICAL ROBOT -> MOVE TO HOME POSE")
    print("=" * 72)

    # 1. Load Target Home
    home_cmd = load_home_target()
    print("Target Home Position (from home_position.txt):")
    for j in ARM_JOINTS:
        print(f"  * {j:<16}: {home_cmd[f'{j}.pos']:>7.2f}°")
    print(f"  * {'gripper':<16}: {home_cmd['gripper.pos']:>7.2f} norm")
    print("-" * 72)

    # 2. Connect Hardware
    robot = connect_robot(port=args.port)
    if robot is None:
        sys.exit(f"[ERROR] Could not connect to robot on {args.port}. Check USB and 12V power supply.")

    # 3. Read Current Position
    cur_obs = robot.get_observation()
    cur_pos = {k: float(cur_obs[k]) for k in cur_obs if k.endswith(".pos")}

    print("Current Robot Encoder Position:")
    for j in ARM_JOINTS:
        print(f"  * {j:<16}: {cur_pos.get(f'{j}.pos', 0.0):>7.2f}°")
    print(f"  * {'gripper':<16}: {cur_pos.get('gripper.pos', 50.0):>7.2f} norm")
    print("-" * 72)

    # 4. Smooth Interpolation to Home
    steps = max(20, int(args.duration * 30.0))  # 30 Hz interpolation
    step_dt = args.duration / float(steps)

    print(f"[*] Smoothly slewing arm to Home over {args.duration:.1f}s ({steps} steps)...")
    for s in range(1, steps + 1):
        alpha = s / float(steps)
        interp = {}
        for k in home_cmd:
            start_v = cur_pos.get(k, home_cmd[k])
            end_v = home_cmd[k]
            interp[k] = round(start_v + alpha * (end_v - start_v), 2)

        robot.send_action(interp)
        time.sleep(step_dt)

    time.sleep(0.3)  # Settling delay

    # 5. Verify Arrival
    final_obs = robot.get_observation()
    print("\n" + "=" * 72)
    print("               HOME POSITION ARRIVAL VERIFICATION")
    print("=" * 72)
    print(f" {'Joint Name':<18} | {'Target':<10} | {'Actual':<10} | {'Error':<10} | {'Status':<10}")
    print("-" * 72)

    all_reached = True
    for j in ARM_JOINTS:
        k = f"{j}.pos"
        t_val = home_cmd[k]
        a_val = float(final_obs.get(k, 0.0))
        err = abs(t_val - a_val)
        status = "[REACHED]" if err <= 3.0 else "[OFFSET]"
        if err > 3.0: all_reached = False
        print(f" {j.replace('_', ' ').title():<18} | {t_val:>7.2f}°  | {a_val:>7.2f}°  | {err:>7.2f}°  | {status}")

    t_g = home_cmd["gripper.pos"]
    a_g = float(final_obs.get("gripper.pos", 50.0))
    err_g = abs(t_g - a_g)
    status_g = "[REACHED]" if err_g <= 5.0 else "[OFFSET]"
    if err_g > 5.0: all_reached = False
    print(f" {'Gripper':<18} | {t_g:>7.2f}   | {a_g:>7.2f}   | {err_g:>7.2f}   | {status_g}")
    print("=" * 72)

    if all_reached:
        print("[SUCCESS] Robot successfully reached and verified at Home Pose!\n")
    else:
        print("[WARNING] Minor joint offset detected from calibrated target.\n")

    # 6. Torque Management
    if args.limp:
        turn_off = True
    else:
        try:
            ans = input("Do you want to turn OFF the robot's torque? (y/N): ").strip().lower()
            turn_off = ans in ("y", "yes")
        except (KeyboardInterrupt, EOFError):
            turn_off = False

    if turn_off:
        print("[*] Disabling motor torque on all servos...")
        try:
            if hasattr(robot, "bus"):
                if hasattr(robot.bus, "disable_torque"):
                    robot.bus.disable_torque()
                elif hasattr(robot.bus, "write"):
                    for m_id in range(1, 7):
                        try: robot.bus.write("Torque_Enable", 0, m_id)
                        except Exception: pass
        except Exception as e:
            print(f"[!] Warning disabling torque: {e}")
        try: robot.disconnect()
        except Exception: pass
        print("[OK] Torque is OFF. Robot arm is limp (unpowered).")
    else:
        print("[*] Torque remains ACTIVE. Robot is holding Home position firmly.")
        try: robot.disconnect()
        except Exception: pass


if __name__ == "__main__":
    main()

