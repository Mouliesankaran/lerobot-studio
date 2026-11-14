#!/usr/bin/env python3
"""Dedicated Interactive Teleoperation Suite for SO-101 Follower Robot Arm.

Features:
- Direct keyboard joint teleoperation via Linux terminal.
- Supports both PHYSICAL robot hardware and SOFTWARE SIMULATION mode (--sim).
- In --sim mode, sends real-time UDP commands to mirror_sim.py.
- Joint selection (1-5 for arm, 6 for gripper).
- Smooth jogging (+ / - or Up / Down arrows).
- Quick gripper controls ('o' open, 'c' close).
- Step size adjustment ('[' decrease, ']' increase).
- 'h' : Smooth return to calibrated Home position.
- 't' : Toggle Torque ON / OFF (Manual lead-through hand guiding).
- 'q' or Ctrl+C : Safely returns robot to Home position, turns OFF torque, and exits.
"""

import argparse
import json
import os
import select
import socket
import sys
import termios
import time
import tty
from pathlib import Path

# Clean ROS Humble path pollution
sys.path = [p for p in sys.path if not p.startswith("/opt/ros")]

CURR_DIR = Path(__file__).resolve().parent
HOME_FILE = CURR_DIR / "home_position.txt"
ZERO_FILE = CURR_DIR / "zero_position.txt"

def get_default_port():
    if "ROBOT_PORT" in os.environ:
        return os.environ["ROBOT_PORT"]
    cands = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
    return cands[0] if cands else "/dev/ttyACM0"

PORT = get_default_port()
DEFAULT_ROBOT_ID = "my_awesome_follower_arm"
TELEOP_UDP_PORT = 8765

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ALL_JOINTS = ARM_JOINTS + ["gripper"]

# Jog directions (+ moves UP/positive, - moves DOWN/negative)
JOG_DIRECTIONS = {
    "shoulder_pan": 1.0,
    "shoulder_lift": 1.0,
    "elbow_flex": -1.0,
    "wrist_flex": -1.0,
    "wrist_roll": 1.0,
    "gripper": 1.0,
}


# --------------------------------------------------------------------------- #
# Safety helpers: joint hard stops + CORRECT LeRobot torque control
# --------------------------------------------------------------------------- #
JOINT_LIMITS = {
    "shoulder_pan": (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex": (-97.0, 97.0),  # home pose uses 96.83, so a 96.8 limit would alter it
    "wrist_flex": (-95.0, 95.0),
    "wrist_roll": (-88.0, 88.0),
    "gripper": (0.0, 100.0),
}


def clamp_joint(name, value):
    lo, hi = JOINT_LIMITS.get(name.removesuffix(".pos"), (-1e9, 1e9))
    return float(min(max(value, lo), hi))


def clamp_action(action):
    return {k: round(clamp_joint(k, v), 2) for k, v in action.items()}


def set_torque(robot, enable):
    """LeRobot bus.write() is write(data_name, MOTOR, VALUE); use the bus helpers instead."""
    bus = getattr(robot, "bus", None)
    if bus is None:
        return False
    try:
        if enable:
            bus.enable_torque()
        else:
            bus.disable_torque()
        return True
    except Exception as e:
        print(f"[!] Torque {'ON' if enable else 'OFF'} failed: {e}")
        return False


def torque_on_safely(robot):
    """Write goal = present position first so the arm does not jump to a stale goal, then enable torque."""
    try:
        obs = robot.get_observation()
        robot.send_action({k: float(v) for k, v in obs.items() if k.endswith(".pos")})
    except Exception:
        pass
    return set_torque(robot, True)


GRIPPER_OPEN = 0.0      # NOTE: LeRobot normally uses 0 = closed, 100 = open. Swap these two if 'o' closes your gripper.
GRIPPER_CLOSED = 100.0
STEP_CYCLE = [1.0, 2.0, 5.0]
MAX_SLEW_DEG = 4.0      # max change per 30 Hz tick (~120 deg/s) so 'h' glides instead of jumping
MAX_SLEW_GRIP = 8.0


def slew_toward(sent, target):
    out = {}
    for k, tv in target.items():
        cur = sent.get(k, tv)
        lim = MAX_SLEW_GRIP if k.startswith("gripper") else MAX_SLEW_DEG
        out[k] = round(cur + max(-lim, min(lim, tv - cur)), 2)
    return out


class TerminalKeyboard:
    """Non-blocking keyboard reader. Uses raw fd reads: buffered sys.stdin.read() swallowed the
    2nd/3rd byte of arrow-key sequences, which then showed up as a stray '[' (step size change)."""

    ESC_MAP = {"A": "UP", "B": "DOWN", "C": "RIGHT", "D": "LEFT"}

    def __init__(self):
        self.is_tty = sys.stdin.isatty()
        self.old_settings = None
        self.fd = sys.stdin.fileno() if self.is_tty else None
        self.queue = []
        if self.is_tty:
            try:
                self.old_settings = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
                sys.stdout.write("\033[?25l")
                sys.stdout.flush()
            except Exception:
                self.is_tty = False

    def _parse(self, data):
        i = 0
        while i < len(data):
            if data[i] == "\x1b" and i + 2 < len(data) and data[i + 1] in "[O":
                name = self.ESC_MAP.get(data[i + 2])
                if name:
                    self.queue.append(name)
                i += 3
            else:
                self.queue.append(data[i])
                i += 1

    def get_key(self):
        if not self.is_tty:
            return None
        if not self.queue and select.select([self.fd], [], [], 0)[0]:
            try:
                self._parse(os.read(self.fd, 64).decode("utf-8", errors="ignore"))
            except OSError:
                return None
        return self.queue.pop(0) if self.queue else None

    def restore(self):
        if self.is_tty and self.old_settings is not None:
            try:
                sys.stdout.write("\033[?25h\n")
                sys.stdout.flush()
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass


def load_home_target():
    """Reads calibrated target from home_position.txt or zero_position.txt."""
    target_file = HOME_FILE if HOME_FILE.is_file() else ZERO_FILE
    pos = {}
    if target_file.is_file():
        try:
            for line in target_file.read_text().splitlines():
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                if ":" in line:
                    k, v = line.split(":", 1)
                    k = k.strip()
                    if not k.endswith(".pos"):
                        k = f"{k}.pos"
                    pos[k] = float(v.strip())
            return pos
        except Exception:
            pass
    return {f"{j}.pos": 0.0 for j in ALL_JOINTS}


class SimulatedTeleopBridge:
    """Sends teleop commands to mirror_sim.py via local UDP socket."""

    def __init__(self, initial_pos=None):
        self.is_connected = True
        self.pos = initial_pos.copy() if initial_pos else {f"{j}.pos": 0.0 for j in ALL_JOINTS}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = ("127.0.0.1", TELEOP_UDP_PORT)
        self.bus = self

    def write(self, param, value, motor_id):
        pass

    def enable_torque(self, *a, **k):
        pass

    def disable_torque(self, *a, **k):
        pass

    def get_observation(self):
        return self.pos.copy()

    def send_action(self, action):
        for k, v in action.items():
            if k in self.pos:
                self.pos[k] = float(v)
        try:
            pkt = json.dumps(action).encode("utf-8")
            self.sock.sendto(pkt, self.addr)
        except Exception:
            pass

    def disconnect(self):
        try: self.sock.close()
        except Exception: pass
        self.is_connected = False


def connect_robot(port=PORT, robot_id=None, is_sim=False, max_attempts=3):
    """Connects to LeRobot physical follower arm or Software Simulation bridge."""
    home_pos = load_home_target()
    if is_sim:
        print("[*] Starting Teleoperation in SOFTWARE SIMULATION mode (--sim)...")
        print("    Commands will be bridged to mirror_sim.py via local UDP port 8765.")
        return SimulatedTeleopBridge(initial_pos=home_pos)

    if robot_id is None:
        if "ROBOT_ID" in os.environ:
            robot_id = os.environ["ROBOT_ID"]
        else:
            calib_dir = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "robots" / "so_follower"
            cands = list(calib_dir.glob("*.json")) if calib_dir.is_dir() else []
            robot_id = cands[0].stem if cands else DEFAULT_ROBOT_ID

    print(f"[*] Connecting to physical SO-101 arm on {port} (id='{robot_id}')...")
    last_err = ""
    for attempt in range(1, max_attempts + 1):
        for cfg in [
            {"port": port, "id": robot_id, "use_degrees": True, "max_relative_target": 35.0},
            {"port": port, "id": robot_id, "use_degrees": True},
        ]:
            try:
                from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
                cand = SO101Follower(SO101FollowerConfig(**cfg))
                cand.connect(calibrate=False)
                if getattr(cand, "is_connected", False) or (hasattr(cand, "bus") and getattr(cand.bus, "is_connected", False)):
                    set_torque(cand, True)
                    print(f"    [OK] Connected successfully on {port} (id='{robot_id}')")
                    return cand
            except Exception as e:
                last_err = str(e)
                try: cand.disconnect()
                except Exception: pass

        if attempt < max_attempts:
            print(f"    [!] Connect attempt {attempt}/{max_attempts} failed: {last_err}. Retrying in 1.0s...")
            time.sleep(1.0)

    print(f"\n[!] Physical robot not detected on {port} ({last_err}).")
    ans = input("Would you like to start in SOFTWARE SIMULATION mode (--sim)? (Y/n): ").strip().lower()
    if ans in ("", "y", "yes"):
        return SimulatedTeleopBridge(initial_pos=home_pos)

    sys.exit(f"[ERROR] Could not connect to physical robot: {last_err}")


def render_teleop_dashboard(real_pos, current_cmd, selected_idx, step_deg, torque_enabled, status_msg, is_sim=False):
    """Draws a clean in-place teleoperation status dashboard."""
    lines = []
    lines.append("=" * 80)
    mode_tag = "SOFTWARE SIMULATION MODE (--sim)" if is_sim else "PHYSICAL HARDWARE MODE"
    lines.append(f"          SO-101 INTERACTIVE TELEOPERATION ({mode_tag})")
    lines.append("=" * 80)

    t_str = "ACTIVE (Motors Holding & Driving)" if torque_enabled else "OFF (Manual Lead-Through Hand Guide)"
    lines.append(f" Torque State : {t_str:<42} | Jog Step: {step_deg:.1f}°")
    lines.append("-" * 80)
    lines.append(f" {'#':<2} | {'Joint Name':<16} | {'Target Command':<16} | {'Real Encoder (Obs)':<18} | {'Diff'}")
    lines.append("-" * 80)

    for idx, j in enumerate(ARM_JOINTS):
        k = f"{j}.pos"
        c_v = current_cmd.get(k, 0.0)
        r_v = real_pos.get(k, 0.0)
        diff = abs(c_v - r_v)
        prefix = "👉" if idx == selected_idx else "  "
        j_display = j.replace("_", " ").title()
        lines.append(f" {prefix}{idx+1} | {j_display:<16} | {c_v:>12.2f}°    | {r_v:>14.2f}°     | {diff:>5.2f}°")

    # Gripper
    c_g = current_cmd.get("gripper.pos", 50.0)
    r_g = real_pos.get("gripper.pos", 50.0)
    g_diff = abs(c_g - r_g)
    g_prefix = "👉" if selected_idx == 5 else "  "
    lines.append(f" {g_prefix}6 | {'Gripper':<16} | {c_g:>12.2f} %    | {r_g:>14.2f} %     | {g_diff:>5.2f}%")

    lines.append("=" * 80)
    sel_name = ALL_JOINTS[selected_idx].replace("_", " ").title()
    lines.append(f" Active Joint : Joint {selected_idx + 1} ({sel_name}) | Info: {status_msg}")
    lines.append("-" * 80)
    lines.append(" Controls: [1-5]: Arm Joints | [6]: Gripper | [Left/Right]: Switch Joint")
    lines.append("           [+/- or Up/Down]: Jog Joint | [o/c]: Gripper Open/Close")
    lines.append("           [[ / ]]: Adjust Step Size (1°, 2°, 5°) | [t]: Torque ON/OFF")
    lines.append("           [h]: Return to Home | [q or Ctrl+C]: Return Home, Torque OFF & Quit")
    lines.append("=" * 80)

    output = "\033[H" + "\n".join(f"\033[K{line}" for line in lines) + "\n\033[J"
    sys.stdout.write(output)
    sys.stdout.flush()


def return_home_and_exit(robot, home_pos, steps=45, step_dt=0.03):
    """Smoothly returns robot to home and turns off torque."""
    print("\n" + "=" * 80)
    print("                 RETURNING PHYSICAL ROBOT SAFELY TO HOME")
    print("=" * 80)
    torque_on_safely(robot)  # if torque was toggled OFF, the arm could not move home before
    try:
        cur_obs = robot.get_observation()
        start_pos = {k: float(cur_obs[k]) for k in cur_obs if k.endswith(".pos")}
        print(f"[*] Gliding arm back to Home position ({steps} steps)...")
        for s in range(1, steps + 1):
            alpha = s / float(steps)
            interp = {k: round(start_pos[k] + alpha * (home_pos[k] - start_pos[k]), 2) for k in home_pos}
            robot.send_action(clamp_action(interp))
            time.sleep(step_dt)
        time.sleep(0.2)
        print("[OK] Real robot arrived at Home pose.")

        # Glide to gravity-safe fold pose (Home is a ~1.4cm hover over the table;
        # cutting torque there lets the arm drop/slam instead of resting on the desk)
        cur_obs = robot.get_observation()
        fold_start = {k: float(v) for k, v in cur_obs.items() if k.endswith(".pos")}
        fold_cmd = dict(fold_start)
        fold_cmd["shoulder_lift.pos"] = -50.0
        fold_cmd["elbow_flex.pos"] = 30.0
        fold_cmd["wrist_flex.pos"] = 0.0
        print("[*] Gliding to gravity-safe fold pose before torque release (40 steps)...")
        for s in range(1, 41):
            alpha = s / 40.0
            interp = {k: fold_start.get(k, fold_cmd[k]) + alpha * (fold_cmd[k] - fold_start.get(k, fold_cmd[k])) for k in fold_cmd}
            robot.send_action(clamp_action(interp))
            time.sleep(step_dt)
        time.sleep(0.2)
        print("[OK] Arm resting on desk in folded pose.")
    except Exception as e:
        print(f"[!] Warning returning home: {e}")

    print("\n[*] Disabling motor torque on all servos (arm will become limp)...")
    if set_torque(robot, False):
        print("[OK] Motor torque is OFF. Robot is safely unpowered.")

    try:
        robot.disconnect()
        print("[OK] Disconnected safely.\n")
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=PORT, help=f"Serial port (default: {PORT})")
    ap.add_argument("--robot-id", default=None, help="Robot ID (auto-detected by default)")
    ap.add_argument("--sim", action="store_true", help="Software simulation mode (bridges to mirror_sim.py via UDP)")
    args = ap.parse_args()

    robot = connect_robot(port=args.port, robot_id=args.robot_id, is_sim=args.sim)
    home_pos = clamp_action(load_home_target())

    cur_obs = robot.get_observation()
    current_cmd = {k: round(float(cur_obs[k]), 2) for k in cur_obs if k.endswith(".pos")}
    sent_cmd = current_cmd.copy()

    selected_idx = 0
    step_idx = 1
    step_deg = STEP_CYCLE[step_idx]
    step_grip = 5.0
    torque_enabled = True
    status_msg = "Ready. Use arrows or hotkeys to jog."

    term_kb = TerminalKeyboard()
    rate_hz = 30.0
    dt = 1.0 / rate_hz

    try:
        while True:
            t0 = time.time()
            key = term_kb.get_key()
            if key is not None:
                if key in ("1", "2", "3", "4", "5", "6"):
                    selected_idx = int(key) - 1
                    status_msg = f"Selected Joint {selected_idx + 1}"
                elif key == "RIGHT":
                    selected_idx = (selected_idx + 1) % len(ALL_JOINTS)
                    status_msg = f"Selected Joint {selected_idx + 1}"
                elif key == "LEFT":
                    selected_idx = (selected_idx - 1) % len(ALL_JOINTS)
                    status_msg = f"Selected Joint {selected_idx + 1}"
                elif key in ("+", "=", "UP", "-", "_", "DOWN"):
                    sign = 1.0 if key in ("+", "=", "UP") else -1.0
                    j_name = ALL_JOINTS[selected_idx]
                    k = f"{j_name}.pos"
                    step = step_grip if selected_idx == 5 else (step_deg * JOG_DIRECTIONS.get(j_name, 1.0))
                    current_cmd[k] = round(clamp_joint(k, current_cmd.get(k, 0.0) + sign * step), 2)
                    status_msg = f"Jogged {j_name} [{'+' if sign > 0 else '-'}] -> {current_cmd[k]:+.2f}"
                elif key in ("o", "O"):
                    current_cmd["gripper.pos"] = GRIPPER_OPEN
                    status_msg = f"Gripper open ({GRIPPER_OPEN:.0f}%)"
                elif key in ("c", "C"):
                    current_cmd["gripper.pos"] = GRIPPER_CLOSED
                    status_msg = f"Gripper close ({GRIPPER_CLOSED:.0f}%)"
                elif key == "]":
                    step_idx = min(len(STEP_CYCLE) - 1, step_idx + 1)
                    step_deg = STEP_CYCLE[step_idx]
                    status_msg = f"Jog step size: {step_deg:.1f}°"
                elif key == "[":
                    step_idx = max(0, step_idx - 1)
                    step_deg = STEP_CYCLE[step_idx]
                    status_msg = f"Jog step size: {step_deg:.1f}°"
                elif key in ("h", "H"):
                    current_cmd = home_pos.copy()
                    status_msg = "Gliding to Home position"
                elif key in ("t", "T"):
                    if torque_enabled:
                        torque_enabled = False
                        set_torque(robot, False)
                    else:
                        torque_on_safely(robot)  # goal = present first, then torque ON (no jump)
                        torque_enabled = True
                        sent_cmd = current_cmd.copy()
                    status_msg = f"Torque: {'ON' if torque_enabled else 'OFF (Manual Hand Guide Mode)'}"
                elif key in ("q", "Q", "\x03"):  # 'q' or Ctrl+C
                    break

            if torque_enabled:
                sent_cmd = slew_toward(sent_cmd, clamp_action(current_cmd))
                robot.send_action(sent_cmd)

            raw_obs = robot.get_observation()
            real_pos = {k: float(raw_obs[k]) for k in raw_obs if k.endswith(".pos")}
            if not torque_enabled:
                current_cmd = {k: round(v, 2) for k, v in real_pos.items()}

            render_teleop_dashboard(
                real_pos=real_pos,
                current_cmd=current_cmd,
                selected_idx=selected_idx,
                step_deg=step_deg,
                torque_enabled=torque_enabled,
                status_msg=status_msg,
                is_sim=args.sim,
            )

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        term_kb.restore()
        return_home_and_exit(robot, home_pos)


if __name__ == "__main__":
    main()