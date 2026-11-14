#!/usr/bin/env python3
"""
SO101 STS3215 Hardware Driver
Provides high-performance synchronous read/write interface to 6x Feetech STS3215 servos.
"""

import time
import math
import logging
from typing import Dict, List, Optional, Tuple, Any

import serial

try:
    import rustypot
    RUSTYPOT_AVAILABLE = True
except ImportError:
    RUSTYPOT_AVAILABLE = False

try:
    import scservo_sdk as scs
    SCSERVO_AVAILABLE = True
except ImportError:
    SCSERVO_AVAILABLE = False

logger = logging.getLogger("SO101Driver")

# Default SO101 joint calibration (from physical calibration so101_calibration.json)
DEFAULT_CALIBRATION = {
    1: {"name": "shoulder_pan",  "id": 1, "homing_offset": 1823, "min": 1197, "max": 3113, "rad_min": -1.4678, "rad_max": 1.4678, "deg_min": -84.1, "deg_max": 84.1},
    2: {"name": "shoulder_lift", "id": 2, "homing_offset": 985,  "min": 1,    "max": 4091, "rad_min": -3.1398, "rad_max": 3.1398, "deg_min": -179.9, "deg_max": 179.9},
    3: {"name": "elbow_flex",    "id": 3, "homing_offset": -327, "min": 0,    "max": 4094, "rad_min": -3.1416, "rad_max": 3.1398, "deg_min": -180.0, "deg_max": 179.9},
    4: {"name": "wrist_flex",    "id": 4, "homing_offset": 1874, "min": 1102, "max": 3022, "rad_min": -1.4713, "rad_max": 1.4713, "deg_min": -84.3, "deg_max": 84.3},
    5: {"name": "wrist_roll",    "id": 5, "homing_offset": 601,  "min": 0,    "max": 4095, "rad_min": -3.1416, "rad_max": 3.1416, "deg_min": -180.0, "deg_max": 180.0},
    6: {"name": "gripper",       "id": 6, "homing_offset": 1467, "min": 2013, "max": 3505, "rad_min": 0.0,    "rad_max": 1.0,    "deg_min": 0.0,   "deg_max": 100.0},
}

STS_ENCODER_RESOLUTION = 4096


def calc_sts_checksum(pkt: bytearray) -> int:
    return (~sum(pkt[2:])) & 0xFF


class SO101Driver:
    """Low-level / high-level STS3215 driver for SO101 robotic arm."""

    def __init__(self, port: str = "/dev/ttyACM0", baudrate: int = 1000000, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.calibration = DEFAULT_CALIBRATION
        self.is_connected = False

        self._ser: Optional[serial.Serial] = None
        self._bus: Optional[Any] = None
        self._defn: Optional[Any] = None

    def connect(self, port: Optional[str] = None, baudrate: Optional[int] = None) -> bool:
        """Connect to the bus dynamically."""
        if port:
            self.port = port
        if baudrate:
            self.baudrate = baudrate

        if self.is_connected:
            self.disconnect()
        if RUSTYPOT_AVAILABLE:
            try:
                _, defn = rustypot.find_model("sts3215")
                self._defn = defn
                defs = {sid: defn for sid in self.calibration.keys()}
                self._bus = rustypot.Bus(self.port, self.baudrate, self.timeout, defs)
                self.is_connected = True
                logger.info(f"Connected to SO101 on {self.port} at {self.baudrate} baud via rustypot.")
                return True
            except Exception as e:
                logger.warning(f"rustypot connection failed ({e}), falling back to direct serial...")

        # Fallback to direct serial
        try:
            self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self.is_connected = True
            logger.info(f"Connected to SO101 on {self.port} at {self.baudrate} baud via raw serial.")
            return True
        except Exception as ex:
            logger.error(f"Failed to open serial port {self.port}: {ex}")
            self.is_connected = False
            return False

    def disconnect(self):
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self.is_connected = False
        logger.info("Disconnected from SO101.")

    def ping_servo(self, servo_id: int) -> bool:
        """Ping an individual servo ID."""
        if not self.is_connected:
            return False
        if self._bus is not None:
            try:
                val = self._bus.read_register(servo_id, "model_number")
                return val == 777
            except Exception:
                return False
        elif self._ser is not None:
            pkt = bytearray([0xFF, 0xFF, servo_id, 2, 0x01])
            pkt.append(calc_sts_checksum(pkt))
            self._ser.reset_input_buffer()
            self._ser.write(pkt)
            resp = self._ser.read(6)
            return len(resp) >= 6 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == servo_id
        return False

    def read_servo_feedback(self, servo_id: int) -> Optional[Dict[str, Any]]:
        """Read all available feedback registers for an individual servo."""
        if not self.is_connected:
            return None

        cal = self.calibration.get(servo_id, {})
        joint_name = cal.get("name", f"joint_{servo_id}")

        if self._bus is not None:
            try:
                pos_raw = self._bus.read_register(servo_id, "present_position")
                vel_raw = self._bus.read_register(servo_id, "present_velocity")
                load = self._bus.read_register(servo_id, "present_load")
                volt = self._bus.read_register(servo_id, "present_voltage") / 10.0
                temp = self._bus.read_register(servo_id, "present_temperature")
                status = self._bus.read_register(servo_id, "status")

                # Convert encoder ticks to radians
                pos_rad, pos_deg = self.ticks_to_angles(servo_id, pos_raw)
                vel_rad = self.raw_speed_to_rad_s(vel_raw)

                return {
                    "id": servo_id,
                    "joint_name": joint_name,
                    "encoder_ticks": pos_raw,
                    "position_rad": pos_rad,
                    "position_deg": pos_deg,
                    "velocity_raw": vel_raw,
                    "velocity_rad_s": vel_rad,
                    "load": load,
                    "voltage_v": volt,
                    "temperature_c": temp,
                    "status": status,
                    "error_alarm": status != 0
                }
            except Exception:
                return None

        elif self._ser is not None:
            # Read 8 bytes starting from address 56 (present_position 2B, present_velocity 2B, present_load 2B, volt 1B, temp 1B)
            pkt = bytearray([0xFF, 0xFF, servo_id, 4, 0x02, 56, 8])
            pkt.append(calc_sts_checksum(pkt))
            self._ser.reset_input_buffer()
            self._ser.write(pkt)
            resp = self._ser.read(14)
            if len(resp) >= 14 and resp[0] == 0xFF and resp[1] == 0xFF and resp[2] == servo_id:
                data = resp[5:13]
                pos_raw = int.from_bytes(data[0:2], "little", signed=True)
                vel_raw = int.from_bytes(data[2:4], "little", signed=True)
                load = int.from_bytes(data[4:6], "little", signed=True)
                volt = data[6] / 10.0
                temp = data[7]

                pos_rad, pos_deg = self.ticks_to_angles(servo_id, pos_raw)
                vel_rad = self.raw_speed_to_rad_s(vel_raw)

                return {
                    "id": servo_id,
                    "joint_name": joint_name,
                    "encoder_ticks": pos_raw,
                    "position_rad": pos_rad,
                    "position_deg": pos_deg,
                    "velocity_raw": vel_raw,
                    "velocity_rad_s": vel_rad,
                    "load": load,
                    "voltage_v": volt,
                    "temperature_c": temp,
                    "status": 0,
                    "error_alarm": False
                }
        return None

    def read_all_servos(self) -> Dict[int, Dict[str, Any]]:
        """Read telemetry feedback from all 6 servos."""
        result = {}
        for sid in self.calibration.keys():
            data = self.read_servo_feedback(sid)
            if data is not None:
                result[sid] = data
        return result

    def set_torque(self, enable: bool, servo_ids: Optional[List[int]] = None) -> bool:
        """Enable or disable motor torque (for manual lead-through movement)."""
        if not self.is_connected:
            return False
        sids = servo_ids if servo_ids is not None else list(self.calibration.keys())
        val = 1 if enable else 0
        success = True
        for sid in sids:
            if self._bus is not None:
                try:
                    self._bus.write_register(sid, "torque_enable", val)
                except Exception as e:
                    logger.warning(f"Failed to set torque {enable} on servo {sid}: {e}")
                    success = False
            elif self._ser is not None:
                pkt = bytearray([0xFF, 0xFF, sid, 4, 0x03, 40, val])
                pkt.append(calc_sts_checksum(pkt))
                try:
                    self._ser.write(pkt)
                except Exception:
                    success = False
        return success

    def ticks_to_angles(self, servo_id: int, ticks: int) -> Tuple[float, float]:
        """Convert raw encoder ticks (0..4095) to calibrated radians and degrees."""
        cal = self.calibration.get(servo_id, {})
        c_min = cal.get("min", 0)
        c_max = cal.get("max", 4095)

        if servo_id == 6:
            # Gripper is normalized 0% (closed) to 100% (open)
            pct = ((ticks - c_min) / max(1, c_max - c_min)) * 100.0
            pct = max(0.0, min(100.0, pct))
            rad = pct / 100.0
            return rad, pct

        mid = (c_min + c_max) / 2.0
        deg = (ticks - mid) * (360.0 / 4095.0)
        rad = math.radians(deg)
        return rad, deg

    def angles_to_ticks(self, servo_id: int, rad: float) -> int:
        """Convert calibrated radians (or gripper 0..100) to raw encoder ticks."""
        cal = self.calibration.get(servo_id, {})
        c_min = cal.get("min", 0)
        c_max = cal.get("max", 4095)

        if servo_id == 6:
            # If rad is in 0..1 (fraction), convert to percentage
            pct = rad if rad > 1.05 else rad * 100.0
            pct = max(0.0, min(100.0, pct))
            ticks = int((pct / 100.0) * (c_max - c_min) + c_min)
            return max(c_min, min(c_max, ticks))

        mid = (c_min + c_max) / 2.0
        deg = math.degrees(rad)
        ticks = int((deg * 4095.0 / 360.0) + mid)
        return max(c_min, min(c_max, ticks))

    def raw_speed_to_rad_s(self, raw_speed: int) -> float:
        """Convert STS3215 raw speed (steps/s) to rad/s."""
        # 4096 steps = 2*pi rad
        return raw_speed * (2.0 * math.pi / STS_ENCODER_RESOLUTION)

    def write_goal_position(self, servo_id: int, target_rad: float, speed_rad_s: float = 1.0, accel: int = 50) -> bool:
        """Send safe target position command to an individual servo."""
        if not self.is_connected:
            return False

        ticks = self.angles_to_ticks(servo_id, target_rad)
        speed_ticks = max(10, min(3400, int(speed_rad_s * STS_ENCODER_RESOLUTION / (2.0 * math.pi))))

        if self._bus is not None:
            try:
                self._bus.write_register(servo_id, "acceleration", accel)
                self._bus.write_register(servo_id, "goal_velocity", speed_ticks)
                self._bus.write_register(servo_id, "goal_position", ticks)
                return True
            except Exception as e:
                logger.error(f"Error writing goal to servo {servo_id}: {e}")
                return False
        elif self._ser is not None:
            # Write goal_position (addr 42, 2 bytes)
            pkt = bytearray([0xFF, 0xFF, servo_id, 5, 0x03, 42])
            pkt.extend(ticks.to_bytes(2, "little", signed=True))
            pkt.append(calc_sts_checksum(pkt))
            self._ser.write(pkt)
            return True
        return False
