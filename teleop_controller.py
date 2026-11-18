#!/usr/bin/env python3
"""
SO-101 Teleoperation & Motion Safety Controller
Enforces Motor ON / Torque Interlock and Emergency Stop (E-STOP) safety states.
"""

import threading
from typing import Dict, Any, Tuple, Optional


class TeleopController:
    """Manages motion safety, Emergency Stop, and manual joint movement."""

    def __init__(self, driver=None):
        self.driver = driver
        self.is_motor_on: bool = True
        self.is_emergency_stopped: bool = False
        self.active_targets: Dict[str, float] = {}
        self.lock = threading.Lock()

    def can_execute_movement(self) -> Tuple[bool, str]:
        """Safety interlock: returns (allowed, reason)."""
        if self.is_emergency_stopped:
            return False, "EMERGENCY STOP is active! Please reset E-STOP and enable motors."
        if not self.is_motor_on:
            return False, "Motor is OFF! Turn Motor ON to move the robot."
        if self.driver is not None and not self.driver.is_connected:
            return False, "Robot hardware is disconnected. Please connect first."
        return True, "OK"

    def set_motor_power(self, enable: bool) -> bool:
        """Enable or disable motor torque."""
        with self.lock:
            if self.is_emergency_stopped and enable:
                print("[SAFETY] Cannot turn motor ON while Emergency Stop is active.")
                return False
            self.is_motor_on = enable
            if self.driver is not None and self.driver.is_connected:
                self.driver.set_torque(enable)
            print(f"[MOTOR] Motor power set to {'ON' if enable else 'OFF'}")
            return True

    def trigger_emergency_stop(self) -> Dict[str, Any]:
        """Immediately halts all motion and cuts torque."""
        with self.lock:
            self.is_emergency_stopped = True
            self.is_motor_on = False
            if self.driver is not None and self.driver.is_connected:
                self.driver.set_torque(False)
            print("[E-STOP] EMERGENCY STOP TRIGGERED! All motions frozen and torque disabled.")
            return {
                "success": True,
                "emergency_stopped": True,
                "motor_on": False,
                "message": "EMERGENCY STOP ACTIVATED. All motion halted."
            }

    def reset_emergency_stop(self) -> Dict[str, Any]:
        """Clear E-STOP condition. User must explicitly re-enable motors."""
        with self.lock:
            self.is_emergency_stopped = False
            print("[E-STOP] Emergency stop cleared. Turn Motor ON to resume operations.")
            return {
                "success": True,
                "emergency_stopped": False,
                "motor_on": self.is_motor_on,
                "message": "Emergency stop cleared. Turn Motor ON to operate."
            }

    def send_target_positions(self, targets: Dict[str, float], velocity: float = 1.5) -> Dict[str, Any]:
        """Send joint angles if safety checks pass."""
        allowed, reason = self.can_execute_movement()
        if not allowed:
            return {"success": False, "message": reason}

        with self.lock:
            self.active_targets.update(targets)
            if self.driver is not None and self.driver.is_connected:
                self.driver.sync_write_positions(targets, velocity=velocity)
            return {"success": True, "targets": dict(self.active_targets)}

    def get_status(self) -> Dict[str, Any]:
        return {
            "motor_on": self.is_motor_on,
            "emergency_stopped": self.is_emergency_stopped,
            "can_move": self.can_execute_movement()[0]
        }
