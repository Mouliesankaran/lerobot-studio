#!/usr/bin/env python3
"""
SO101 Hardware Connection & All Servo Feedback Verification Node (ROS 2 Humble)

Purpose:
- Connects to physical SO101 serial port (/dev/ttyACM0 at 1,000,000 baud).
- Subscribes to /so101/joint_commands [sensor_msgs/JointState] to drive physical servos in real time.
- Continuously reads and verifies all servo feedback parameters:
    * Encoder values (raw ticks: 0-4095)
    * Angular position (radians & degrees)
    * Joint velocity (rad/s)
    * Motor load / current
    * Bus voltage (V) & internal temperature (°C)
    * Status error & alarm flags
- Publishes synchronized joint telemetry to ROS 2 topic: /so101/joint_states
"""

import os
import sys
import time
import math
import json
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from so101_servo_dynamics.so101_driver import SO101Driver, DEFAULT_CALIBRATION


class HardwareConnectionNode(Node):
    def __init__(self):
        super().__init__('so101_hardware_connection')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('enable_sim_fallback', True)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.rate_hz = self.get_parameter('publish_rate_hz').value
        self.sim_fallback = self.get_parameter('enable_sim_fallback').value

        self.get_logger().info("==================================================")
        self.get_logger().info("   SO101 HARDWARE CONNECTION & REAL-TIME DRIVER   ")
        self.get_logger().info("==================================================")
        self.get_logger().info(f"Connecting to port: {self.port} at {self.baudrate} baud...")

        # Initialize hardware driver
        self.driver = SO101Driver(port=self.port, baudrate=self.baudrate)
        self.is_connected = self.driver.connect()

        # Target positions cache for safety & fallback
        self.current_targets = {cal["name"]: 0.0 for cal in DEFAULT_CALIBRATION.values()}
        self.sim_positions = {cal["name"]: 0.0 for cal in DEFAULT_CALIBRATION.values()}

        if not self.is_connected:
            self.get_logger().warn(f"Could not open {self.port}. Virtual simulation fallback active.")
        else:
            self.get_logger().info("Serial port opened successfully. Performing 6-servo handshake...")
            self.verify_initial_handshake()

        # ROS 2 Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/so101/joint_states', 10)
        self.diag_pub = self.create_publisher(String, '/so101/diagnostics', 10)

        # ROS 2 Subscriber: Commands from Web UI or teleoperation
        self.joint_command_sub = self.create_subscription(
            JointState,
            '/so101/joint_commands',
            self.joint_command_callback,
            10
        )

        # Timer for polling and publishing
        period = 1.0 / self.rate_hz
        self.timer = self.create_timer(period, self.timer_callback)
        self.last_display_time = time.time()
        self.packet_count = 0

    def verify_initial_handshake(self):
        """Verify presence of all 6 STS3215 servos on startup."""
        responsive_count = 0
        print("\n--- Initial Servo Discovery ---")
        for sid, cal in DEFAULT_CALIBRATION.items():
            name = cal["name"]
            online = self.driver.ping_servo(sid)
            status_str = "ONLINE (STS3215)" if online else "UNRESPONSIVE"
            print(f"  [ID {sid}] {name:<14} : {status_str}")
            if online:
                responsive_count += 1
        print("-------------------------------\n")
        if responsive_count == 6:
            self.get_logger().info("All 6 STS3215 servos verified and responding!")
        else:
            self.get_logger().warn(f"Discovered {responsive_count}/6 servos. Ensure 12V DC power switch is ON.")

    def joint_command_callback(self, msg: JointState):
        """Receive joint commands from ROS 2 and drive physical robot in real time."""
        name_to_id = {cal["name"]: sid for sid, cal in DEFAULT_CALIBRATION.items()}

        for i, name in enumerate(msg.name):
            if name in name_to_id and i < len(msg.position):
                sid = name_to_id[name]
                cal = DEFAULT_CALIBRATION[sid]
                target_rad = float(msg.position[i])

                # Safety limit clamping
                clamped_rad = max(cal["rad_min"], min(cal["rad_max"], target_rad))
                self.current_targets[name] = clamped_rad

                # Velocity limit
                speed = 1.5
                if msg.velocity and i < len(msg.velocity):
                    speed = max(0.1, min(4.0, float(msg.velocity[i])))

                # Send command directly to physical servo
                if self.driver.is_connected:
                    self.driver.write_goal_position(sid, clamped_rad, speed_rad_s=speed)
                else:
                    # In simulation fallback, smooth transition
                    self.sim_positions[name] = clamped_rad

    def timer_callback(self):
        """Read all servo feedbacks, publish ROS 2 JointState, and display status table."""
        now = self.get_clock().now()
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []
        msg.effort = []

        all_feedback = {}
        if self.driver.is_connected:
            all_feedback = self.driver.read_all_servos()

        diag_data = {
            "is_physical": bool(all_feedback),
            "online_count": len(all_feedback) if all_feedback else 0,
            "servos": {}
        }

        if all_feedback:
            for sid in sorted(all_feedback.keys()):
                fb = all_feedback[sid]
                msg.name.append(fb["joint_name"])
                msg.position.append(fb["position_rad"])
                msg.velocity.append(fb["velocity_rad_s"])
                msg.effort.append(float(fb["load"]))
                diag_data["servos"][fb["joint_name"]] = {
                    "id": sid,
                    "encoder_ticks": fb["encoder_ticks"],
                    "voltage_v": fb["voltage_v"],
                    "temperature_c": fb["temperature_c"],
                    "load": fb["load"],
                    "status": fb["status"]
                }
        elif self.sim_fallback:
            # Provide smooth mock telemetry for testing when hardware power is offline
            for sid, cal in DEFAULT_CALIBRATION.items():
                name = cal["name"]
                target = self.current_targets[name]
                curr = self.sim_positions[name]
                # Smooth interpolation towards target
                curr += 0.25 * (target - curr)
                self.sim_positions[name] = curr

                msg.name.append(name)
                msg.position.append(curr)
                msg.velocity.append((target - curr) * 3.0)
                msg.effort.append(15.0)
                diag_data["servos"][name] = {
                    "id": sid,
                    "encoder_ticks": 2048,
                    "voltage_v": 12.0,
                    "temperature_c": 30,
                    "load": 15,
                    "status": 0
                }

        self.joint_state_pub.publish(msg)
        
        # Publish rich diagnostics JSON
        diag_msg = String()
        diag_msg.data = json.dumps(diag_data)
        self.diag_pub.publish(diag_msg)
        self.packet_count += 1

        # Periodic log output
        if time.time() - self.last_display_time >= 3.0:
            self.last_display_time = time.time()
            active_count = len(all_feedback) if all_feedback else (6 if self.sim_fallback else 0)
            mode = "PHYSICAL" if all_feedback else "SIMULATION_FALLBACK"
            self.get_logger().info(f"[{mode}] Active Servos: {active_count}/6 | Published {self.packet_count} msgs to /so101/joint_states")

    def destroy_node(self):
        self.driver.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareConnectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
