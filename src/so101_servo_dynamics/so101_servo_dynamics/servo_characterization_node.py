#!/usr/bin/env python3
"""
SO101 Servo Characterization Node (ROS 2 Humble)

Purpose:
- Live dynamic measurements of physical STS3215 servos:
    1. Encoder values (raw ticks: 0-4095)
    2. Live Position (radians & degrees)
    3. Live Velocity (rad/s & deg/s)
    4. Live Acceleration (rad/s^2)
    5. Dynamic Response Time Metrics:
        * Dead time / Response delay (Td)
        * Rise time (Tr: 10% to 90%)
        * Settling time (Ts: 2% band)
        * Peak overshoot (%)
        * Steady-state tracking error (ess)
        * Maximum observed velocity & acceleration
- Provides both continuous passive monitoring and controlled step-response characterization.
- Automatically records synchronized high-speed time-series datasets to CSV in data/raw/.
"""

import os
import sys
import time
import math
import csv
from datetime import datetime
from typing import Dict, List, Optional, Tuple, Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from so101_servo_dynamics.so101_driver import SO101Driver, DEFAULT_CALIBRATION, STS_ENCODER_RESOLUTION


class ServoCharacterizationNode(Node):
    def __init__(self):
        super().__init__('so101_servo_characterization')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('sample_rate_hz', 50.0)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.sample_rate = self.get_parameter('sample_rate_hz').value

        # Initialize Hardware Driver
        self.driver = SO101Driver(port=self.port, baudrate=self.baudrate)
        self.is_connected = self.driver.connect()

        # State tracking for numerical differentiation
        self.last_time = time.time()
        self.last_positions: Dict[int, float] = {}   # sid -> rad
        self.last_velocities: Dict[int, float] = {}  # sid -> rad/s
        self.current_accel: Dict[int, float] = {}    # sid -> rad/s^2

        # Step response characterization session variables
        self.test_active = False
        self.active_joint_id: Optional[int] = None
        self.step_target_rad: float = 0.0
        self.step_start_time: float = 0.0
        self.step_data_buffer: List[Dict[str, Any]] = []

        # ROS 2 Publisher for characterization JointState
        self.char_state_pub = self.create_publisher(JointState, '/so101/characterization_states', 10)

        self.get_logger().info("==================================================")
        self.get_logger().info("     SO101 SERVO DYNAMICS CHARACTERIZATION NODE   ")
        self.get_logger().info("==================================================")

        # Polling Timer (50 Hz default)
        dt = 1.0 / self.sample_rate
        self.timer = self.create_timer(dt, self.sample_loop)
        self.last_render_time = time.time()

    def sample_loop(self):
        """High-frequency polling loop for live position, velocity, and acceleration."""
        if not self.is_connected:
            return

        now = time.time()
        dt = now - self.last_time
        if dt <= 0.0:
            return

        all_feedback = self.driver.read_all_servos()
        if not all_feedback:
            return

        # Compute numerical velocity and acceleration
        for sid, fb in all_feedback.items():
            current_pos = fb["position_rad"]

            if sid in self.last_positions:
                # Finite difference velocity (filtered)
                inst_vel = (current_pos - self.last_positions[sid]) / dt
                prev_vel = self.last_velocities.get(sid, 0.0)
                # Exponential smoothing filter
                smooth_vel = 0.7 * inst_vel + 0.3 * prev_vel

                # Finite difference acceleration
                inst_accel = (smooth_vel - prev_vel) / dt
                prev_accel = self.current_accel.get(sid, 0.0)
                smooth_accel = 0.6 * inst_accel + 0.4 * prev_accel

                fb["velocity_rad_s"] = smooth_vel
                fb["accel_rad_s2"] = smooth_accel
                self.last_velocities[sid] = smooth_vel
                self.current_accel[sid] = smooth_accel
            else:
                fb["accel_rad_s2"] = 0.0
                self.last_velocities[sid] = fb.get("velocity_rad_s", 0.0)
                self.current_accel[sid] = 0.0

            self.last_positions[sid] = current_pos

        self.last_time = now

        # Publish JointState with calculated velocity and effort/acceleration
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for sid in sorted(all_feedback.keys()):
            fb = all_feedback[sid]
            msg.name.append(fb["joint_name"])
            msg.position.append(fb["position_rad"])
            msg.velocity.append(fb["velocity_rad_s"])
            msg.effort.append(fb["accel_rad_s2"])
        self.char_state_pub.publish(msg)

        # Record step-response data if an active test is running
        if self.test_active and self.active_joint_id in all_feedback:
            t_rel = now - self.step_start_time
            target_fb = all_feedback[self.active_joint_id]
            sample = {
                "rel_time_s": round(t_rel, 4),
                "timestamp": now,
                "servo_id": self.active_joint_id,
                "joint_name": target_fb["joint_name"],
                "target_rad": round(self.step_target_rad, 5),
                "actual_rad": round(target_fb["position_rad"], 5),
                "actual_deg": round(target_fb["position_deg"], 2),
                "encoder_ticks": target_fb["encoder_ticks"],
                "velocity_rad_s": round(target_fb["velocity_rad_s"], 4),
                "accel_rad_s2": round(target_fb["accel_rad_s2"], 3),
                "error_rad": round(self.step_target_rad - target_fb["position_rad"], 5),
                "voltage_v": target_fb["voltage_v"],
                "temperature_c": target_fb["temperature_c"],
                "load": target_fb["load"]
            }
            self.step_data_buffer.append(sample)

        # Render Live Terminal Dashboard every 0.5s when test is not executing
        if not self.test_active and (now - self.last_render_time >= 0.5):
            self.last_render_time = now
            self.render_live_dashboard(all_feedback)

    def render_live_dashboard(self, all_feedback: Dict[int, Dict[str, Any]]):
        """Render live measurements of encoder, position, velocity, and acceleration."""
        os.system("clear" if os.name == "posix" else "cls")
        print("=" * 86)
        print("        SO101 LIVE SERVO DYNAMICS & KINEMATICS CHARACTERIZATION")
        print("=" * 86)
        print(f"Port: {self.port} | Baud: {self.baudrate} | Rate: {self.sample_rate} Hz | Servos Active: {len(all_feedback)}/6")
        print("-" * 86)
        print(f"{'ID':<3} {'Joint Name':<15} {'Encoder':<8} {'Pos(rad)':<10} {'Pos(deg)':<9} {'Vel(rad/s)':<12} {'Accel(rad/s²)':<14} {'Volt':<6} {'Temp':<5}")
        print("-" * 86)

        for sid, cal in DEFAULT_CALIBRATION.items():
            name = cal["name"]
            if sid in all_feedback:
                fb = all_feedback[sid]
                ticks = fb["encoder_ticks"]
                rad = f"{fb['position_rad']:+.3f}"
                deg = f"{fb['position_deg']:+.1f}°"
                vel = f"{fb['velocity_rad_s']:+.2f}"
                acc = f"{fb.get('accel_rad_s2', 0.0):+.2f}"
                volt = f"{fb['voltage_v']:.1f}V"
                temp = f"{fb['temperature_c']}°C"
                print(f"{sid:<3} {name:<15} {ticks:<8} {rad:<10} {deg:<9} {vel:<12} {acc:<14} {volt:<6} {temp:<5}")
            else:
                print(f"{sid:<3} {name:<15} {'--':<8} {'--':<10} {'--':<9} {'--':<12} {'--':<14} {'--':<6} {'--':<5}")

        print("=" * 86)
        print("Publishing to ROS 2: /so101/characterization_states [JointState]")
        print("To run a single-joint step characterization test, enter interactive mode.")
        print("-" * 86)

    def run_step_response_test(self, servo_id: int, step_angle_deg: float = 10.0, duration: float = 2.5):
        """
        Execute a safe step response test on an individual joint and compute:
        - Response time (delay)
        - Rise time (10% to 90%)
        - Settling time (2% band)
        - Peak overshoot
        - Steady-state error
        """
        if not self.driver.is_connected:
            self.get_logger().error("Robot arm is not connected!")
            return None

        cal = DEFAULT_CALIBRATION.get(servo_id)
        if not cal:
            self.get_logger().error(f"Invalid servo ID {servo_id}")
            return None

        # Read initial position
        initial_fb = self.driver.read_servo_feedback(servo_id)
        if not initial_fb:
            self.get_logger().error(f"Could not read initial position of joint {servo_id}")
            return None

        initial_rad = initial_fb["position_rad"]
        target_rad = initial_rad + math.radians(step_angle_deg)

        # Check safety limits
        if target_rad < cal["rad_min"] or target_rad > cal["rad_max"]:
            self.get_logger().warn(f"Target {target_rad:.2f} rad exceeds joint limits [{cal['rad_min']}, {cal['rad_max']}]. Aborting.")
            return None

        self.get_logger().info(f"Starting step test on ID {servo_id} ({cal['name']}): Initial={math.degrees(initial_rad):.1f}°, Target={math.degrees(target_rad):.1f}°")

        self.step_data_buffer = []
        self.active_joint_id = servo_id
        self.step_target_rad = target_rad
        self.step_start_time = time.time()
        self.test_active = True

        # Issue command to servo
        self.driver.write_goal_position(servo_id, target_rad, speed_rad_s=2.0)

        # Wait for duration of experiment
        t_end = time.time() + duration
        while time.time() < t_end:
            time.sleep(0.01)

        self.test_active = False

        # Compute dynamic characterization metrics
        metrics = self.analyze_step_response(initial_rad, target_rad, self.step_data_buffer)

        # Save dataset to CSV
        csv_path = self.save_dataset_csv(servo_id, cal["name"], self.step_data_buffer)
        metrics["csv_path"] = csv_path

        # Return to initial position safely
        time.sleep(0.5)
        self.driver.write_goal_position(servo_id, initial_rad, speed_rad_s=1.0)
        time.sleep(1.0)

        return metrics

    def analyze_step_response(self, initial_pos: float, target_pos: float, data: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Compute rise time, settling time, delay, overshoot, and steady-state error."""
        if not data:
            return {"status": "NO_DATA"}

        delta_total = target_pos - initial_pos
        if abs(delta_total) < 1e-4:
            return {"status": "DELTA_TOO_SMALL"}

        p10 = initial_pos + 0.10 * delta_total
        p90 = initial_pos + 0.90 * delta_total
        band = 0.02 * abs(delta_total)

        t_start = data[0]["rel_time_s"]
        t_delay = None
        t_10 = None
        t_90 = None
        peak_val = initial_pos
        peak_time = 0.0
        settling_time = None
        max_vel = 0.0
        max_acc = 0.0

        for s in data:
            val = s["actual_rad"]
            t = s["rel_time_s"]
            vel = abs(s["velocity_rad_s"])
            acc = abs(s["accel_rad_s2"])

            if vel > max_vel:
                max_vel = vel
            if acc > max_acc:
                max_acc = acc

            # Detect response delay (first significant motion > 2% of delta)
            if t_delay is None and abs(val - initial_pos) >= 0.02 * abs(delta_total):
                t_delay = t - t_start

            # 10% and 90% rise times
            if t_10 is None and (val >= p10 if delta_total > 0 else val <= p10):
                t_10 = t
            if t_90 is None and (val >= p90 if delta_total > 0 else val <= p90):
                t_90 = t

            # Track peak for overshoot
            if (delta_total > 0 and val > peak_val) or (delta_total < 0 and val < peak_val):
                peak_val = val
                peak_time = t

        # Settling time (time after which response never leaves 2% band)
        for i in reversed(range(len(data))):
            if abs(data[i]["actual_rad"] - target_pos) > band:
                settling_time = data[min(i + 1, len(data) - 1)]["rel_time_s"] - t_start
                break

        rise_time = (t_90 - t_10) if (t_10 is not None and t_90 is not None) else None
        overshoot_pct = max(0.0, ((peak_val - target_pos) / delta_total) * 100.0) if delta_total > 0 else max(0.0, ((target_pos - peak_val) / abs(delta_total)) * 100.0)
        final_pos = data[-1]["actual_rad"]
        ss_error = abs(target_pos - final_pos)

        return {
            "response_delay_s": round(t_delay, 4) if t_delay else 0.0,
            "rise_time_s": round(rise_time, 4) if rise_time else None,
            "settling_time_s": round(settling_time, 4) if settling_time else None,
            "overshoot_percent": round(overshoot_pct, 2),
            "steady_state_error_rad": round(ss_error, 5),
            "max_velocity_rad_s": round(max_vel, 3),
            "max_accel_rad_s2": round(max_acc, 3),
            "samples_collected": len(data)
        }

    def save_dataset_csv(self, servo_id: int, joint_name: str, data: List[Dict[str, Any]]) -> str:
        """Save collected step-response data to CSV."""
        base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        out_dir = os.path.join(base_dir, "data", "raw")
        os.makedirs(out_dir, exist_ok=True)

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"characterization_joint_{servo_id}_{joint_name}_{timestamp_str}.csv"
        filepath = os.path.join(out_dir, filename)

        if not data:
            return ""

        headers = list(data[0].keys())
        with open(filepath, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=headers)
            writer.writeheader()
            writer.writerows(data)

        self.get_logger().info(f"Synchronized characterization dataset saved: {filepath}")
        return filepath

    def destroy_node(self):
        self.driver.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoCharacterizationNode()

    # Check for CLI test argument
    if len(sys.argv) > 1 and sys.argv[1] == "--test-joint":
        joint_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1
        angle = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
        time.sleep(1.0)
        metrics = node.run_step_response_test(joint_id, step_angle_deg=angle)
        print("\n" + "=" * 60)
        print(f"       SERVO ID {joint_id} STEP CHARACTERIZATION RESULTS")
        print("=" * 60)
        for k, v in metrics.items():
            print(f"  {k:<28} : {v}")
        print("=" * 60)
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
