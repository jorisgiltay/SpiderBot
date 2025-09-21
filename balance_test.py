#!/usr/bin/env python
"""
Test script for automatic balance control using IMU data.

This script reads IMU attitude data from a Betaflight flight controller and
automatically adjusts individual leg heights to keep the robot level on
uneven surfaces.

Usage:
    python balance_test.py --msp-port COM4 --servo-port COM3
    python balance_test.py --msp-port /dev/ttyAMA0 --servo-port /dev/ttyUSB0
"""

import argparse
import os
import sys
import time
import struct
import threading
import math
from typing import List, Optional, Tuple, Dict

import serial

# Add servo interface to path
HERE = os.path.dirname(__file__)
WEB_IFACE_DIR = os.path.abspath(os.path.join(HERE, "servos", "webInterface"))
if WEB_IFACE_DIR not in sys.path:
    sys.path.append(WEB_IFACE_DIR)

from servo_manager import ServoManager  # type: ignore


# ---------- MSP Protocol Constants ----------
MSP_ATTITUDE = 108   # Attitude data (roll, pitch, yaw)


# ---------- MSP Protocol Helpers ----------
def build_msp_request(command: int, payload: bytes = b"") -> bytes:
    """Build an MSP request packet."""
    header = b"$M<"
    size = len(payload)
    size_b = size.to_bytes(1, "little")
    cmd_b = command.to_bytes(1, "little")
    checksum = size ^ command
    for b in payload:
        checksum ^= b
    csum_b = checksum.to_bytes(1, "little")
    return header + size_b + cmd_b + payload + csum_b


def read_msp_response(ser: serial.Serial, timeout_s: float = 0.2) -> bytes:
    """Read an MSP response packet."""
    buf = bytearray()
    start_time = time.time()
    while time.time() - start_time < timeout_s:
        buf += ser.read(ser.in_waiting or 1)
        # Look for header
        idx = buf.find(b"$M>")
        if idx != -1 and len(buf) >= idx + 5:
            size = buf[idx + 3]
            total_len = 3 + 1 + 1 + size + 1  # header + size + cmd + payload + checksum
            if len(buf) >= idx + total_len:
                return bytes(buf[idx: idx + total_len])
    return bytes(buf)


def parse_msp_attitude(response: bytes) -> Optional[Tuple[int, int, int]]:
    """
    Parse MSP_ATTITUDE response.
    Returns: (roll, pitch, yaw) in 1/10 degree units or None if invalid
    """
    if len(response) < 5:
        return None
    
    size = response[3]
    if size != 6:  # 3 * 2 bytes per int16
        return None
    
    payload = response[5:5 + size]
    if len(payload) != 6:
        return None
    
    # Unpack 3 signed 16-bit integers (little-endian)
    data = struct.unpack('<3h', payload)
    return data


# ---------- Balance Control ----------
class BalanceController:
    """
    Controls robot balance by adjusting individual leg heights based on IMU data.
    """
    
    def __init__(self, servo_manager: ServoManager, manager_lock: threading.Lock):
        self.servo_manager = servo_manager
        self.manager_lock = manager_lock
        self.ranges = self._load_servo_ranges().get("servos", {})
        
        # Leg mapping: (hip_id, knee_id) for each leg
        self.legs = {
            "FL": (1, 2),  # Front Left
            "FR": (3, 4),  # Front Right  
            "RL": (5, 6),  # Rear Left
            "RR": (7, 8),  # Rear Right
        }
        
        # Balance parameters
        self.base_height_mm = 20.0  # Base height when level
        self.max_adjustment_mm = 15.0  # Maximum height adjustment per leg
        self.sensitivity = 0.5  # How aggressively to correct (0.0 to 1.0)
        self.deadband_deg = 1.0  # Don't adjust if tilt is less than this
        
        # Debug tracking
        self.last_positions = {}  # Track last servo positions for debugging
        
        # Servo control parameters
        self.speed = 1500  # Slower for smoother balance adjustments
        self.acc = 30     # Lower acceleration for smoother motion
        self.command_delay = 0.05  # Delay between servo commands (50ms)
        
    def _load_servo_ranges(self) -> Dict:
        """Load servo ranges from JSON file."""
        here = os.path.dirname(__file__)
        path = os.path.abspath(os.path.join(here, "servos", "servo_ranges.json"))
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}
    
    def _clamp_ticks(self, val: int, smin: int, smax: int) -> int:
        """Clamp servo position to valid range - EXACT copy from MotionController."""
        return max(smin, min(smax, val))

    def _servo_mid(self, sid: int) -> int:
        """Get servo midpoint position - EXACT copy from MotionController."""
        s = self.ranges.get(f"servo{sid}", {})
        try:
            return self._clamp_ticks(int(s["mid"]), int(s["min"]), int(s["max"]))
        except Exception:
            return 2048
    
    def _knee_for_height(self, sid: int, height_mm: float) -> int:
        """Convert height in mm to knee servo position - EXACT copy from MotionController."""
        s = self.ranges.get(f"servo{sid}", {})
        try:
            base_mid = int(s["mid"]) ; smin = int(s["min"]) ; smax = int(s["max"]) 
        except Exception:
            base_mid, smin, smax = 2048, 0, 4095
        # mapping: 0mm -> max_ticks (low), 60mm -> base_mid (high)
        max_height_mm = 60.0
        h = max(0.0, min(max_height_mm, float(height_mm)))
        t = h / max_height_mm
        target = int(round((1.0 - t) * smax + t * base_mid))
        return self._clamp_ticks(target, smin, smax)
    
    def _hip_for_height(self, sid: int, height_mm: float) -> int:
        """Convert height in mm to hip servo position (for fine adjustments)."""
        s = self.ranges.get(f"servo{sid}", {})
        try:
            base_mid = int(s["mid"])
            smin = int(s["min"])
            smax = int(s["max"])
        except Exception:
            base_mid, smin, smax = 2048, 0, 4095
        
        # For hips, we use a smaller range for fine height adjustments
        # Map height adjustment to hip position offset
        max_hip_adjust_mm = 10.0  # Maximum hip adjustment range
        h = max(-max_hip_adjust_mm, min(max_hip_adjust_mm, float(height_mm)))
        t = h / max_hip_adjust_mm
        hip_range = (smax - smin) // 4  # Use 1/4 of full range for adjustments
        offset = int(round(t * hip_range))
        target = base_mid + offset
        return max(smin, min(smax, target))
    
    def calculate_leg_adjustments(self, roll_deg: float, pitch_deg: float) -> Dict[str, float]:
        """
        Calculate height adjustments for each leg based on roll and pitch angles.
        
        Args:
            roll_deg: Roll angle in degrees (positive = right side up)
            pitch_deg: Pitch angle in degrees (positive = nose up)
            
        Returns:
            Dictionary mapping leg names to height adjustments in mm
        """
        # Apply deadband
        if abs(roll_deg) < self.deadband_deg and abs(pitch_deg) < self.deadband_deg:
            return {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
        
        # Apply sensitivity scaling
        roll_adj = roll_deg * self.sensitivity
        pitch_adj = pitch_deg * self.sensitivity
        
        # Calculate adjustments for each leg
        # Roll: positive roll means right side is higher, so lower right legs
        # Pitch: positive pitch means front is higher, so lower front legs
        adjustments = {
            "FL": -pitch_adj + roll_adj,   # Front Left: lower if front high, raise if left high
            "FR": -pitch_adj - roll_adj,   # Front Right: lower if front high, lower if right high  
            "RL": +pitch_adj + roll_adj,   # Rear Left: raise if front high, raise if left high
            "RR": +pitch_adj - roll_adj,   # Rear Right: raise if front high, lower if right high
        }
        
        # Clamp adjustments to maximum range
        for leg in adjustments:
            adjustments[leg] = max(-self.max_adjustment_mm, 
                                 min(self.max_adjustment_mm, adjustments[leg]))
        
        return adjustments
    
    def apply_balance_correction(self, roll_deg: float, pitch_deg: float, debug: bool = False) -> bool:
        """
        Apply balance correction by adjusting leg heights.
        
        Args:
            roll_deg: Roll angle in degrees
            pitch_deg: Pitch angle in degrees
            debug: If True, print detailed servo position information
            
        Returns:
            True if correction was applied successfully
        """
        adjustments = self.calculate_leg_adjustments(roll_deg, pitch_deg)
        
        try:
            # Use same servo ID arrays as rc_drive.py
            knee_ids = [2, 4, 6, 8]
            hip_ids = [1, 3, 5, 7]
            
            # Map leg names to knee IDs for adjustments
            leg_to_knee = {"FL": 2, "FR": 4, "RL": 6, "RR": 8}
            
            with self.manager_lock:
                # First, set all hips to mid (like rc_drive.py)
                for hip_id in hip_ids:
                    hip_pos = self._servo_mid(hip_id)
                    success, error = self.servo_manager.write_position(hip_id, hip_pos, self.speed, self.acc)
                    if not success and debug:
                        print(f"    Failed to set hip {hip_id}: {error}")
                    time.sleep(self.command_delay)
                
                # Then set knees with individual adjustments
                for leg_name, adjustment_mm in adjustments.items():
                    if abs(adjustment_mm) < 0.5:  # Skip tiny adjustments
                        continue
                        
                    knee_id = leg_to_knee[leg_name]
                    target_height = self.base_height_mm + adjustment_mm
                    knee_pos = self._knee_for_height(knee_id, target_height)
                    
                    # Debug: show position changes
                    if debug:
                        old_pos = self.last_positions.get(knee_id, 0)
                        pos_change = knee_pos - old_pos
                        print(f"  {leg_name} knee {knee_id}: {old_pos} -> {knee_pos} (Δ{pos_change:+d}) "
                              f"height: {self.base_height_mm:.1f} -> {target_height:.1f}mm")
                        self.last_positions[knee_id] = knee_pos
                    
                    # Send servo command
                    success, error = self.servo_manager.write_position(knee_id, knee_pos, self.speed, self.acc)
                    if not success and debug:
                        print(f"    Failed to write position to servo {knee_id}: {error}")
                    
                    time.sleep(self.command_delay)
            
            return True
        except Exception as e:
            print(f"Error applying balance correction: {e}")
            return False
    
    def set_base_height(self, height_mm: float) -> bool:
        """Set the base height for all legs when level."""
        self.base_height_mm = max(0.0, min(60.0, height_mm))
        return self._set_all_legs_to_height(self.base_height_mm)
    
    def _set_all_legs_to_height(self, height_mm: float) -> bool:
        """Set all legs to the same height - matches rc_drive.py implementation."""
        try:
            # Use same servo ID arrays as rc_drive.py
            knee_ids = [2, 4, 6, 8]
            hip_ids = [1, 3, 5, 7]
            
            with self.manager_lock:
                # Set ALL hips to mid first (like rc_drive.py)
                for hip_id in hip_ids:
                    hip_pos = self._servo_mid(hip_id)
                    success, error = self.servo_manager.write_position(hip_id, hip_pos, self.speed, self.acc)
                    if not success:
                        print(f"Failed to set hip {hip_id} to {hip_pos}: {error}")
                    time.sleep(self.command_delay)
                
                # Then set ALL knees to target height (like rc_drive.py)
                for knee_id in knee_ids:
                    knee_pos = self._knee_for_height(knee_id, height_mm)
                    success, error = self.servo_manager.write_position(knee_id, knee_pos, self.speed, self.acc)
                    if not success:
                        print(f"Failed to set knee {knee_id} to {knee_pos}: {error}")
                    else:
                        self.last_positions[knee_id] = knee_pos
                    time.sleep(self.command_delay)
            
            return True
        except Exception as e:
            print(f"Error setting leg heights: {e}")
            return False
    
    def test_servo_movement(self) -> bool:
        """Test servo movement by making a visible height change."""
        print("Testing servo movement...")
        try:
            # Move to a different height temporarily
            test_height = self.base_height_mm + 10.0
            print(f"Moving to test height: {test_height}mm")
            success = self._set_all_legs_to_height(test_height)
            if success:
                time.sleep(3.0)  # Wait longer for servos to reach test position
                print(f"Returning to base height: {self.base_height_mm}mm")
                success = self._set_all_legs_to_height(self.base_height_mm)
                time.sleep(2.0)  # Wait longer for servos to return
            return success
        except Exception as e:
            print(f"Error in servo test: {e}")
            return False


# ---------- Main Function ----------
def main() -> int:
    parser = argparse.ArgumentParser(description="Test automatic balance control using IMU data")
    parser.add_argument("--msp-port", default="/dev/ttyAMA0", help="MSP/FC serial port")
    parser.add_argument("--msp-baud", type=int, default=115200, help="MSP/FC baud rate")
    parser.add_argument("--servo-port", default=None, help="Servo bus serial port")
    parser.add_argument("--servo-baud", type=int, default=1_000_000, help="Servo bus baud rate")
    parser.add_argument("--base-height", type=float, default=20.0, help="Base height in mm when level")
    parser.add_argument("--max-adjustment", type=float, default=15.0, help="Maximum height adjustment per leg (mm)")
    parser.add_argument("--sensitivity", type=float, default=0.5, help="Balance sensitivity (0.0 to 1.0)")
    parser.add_argument("--deadband", type=float, default=1.0, help="Deadband in degrees (no adjustment below this)")
    parser.add_argument("--rate", type=float, default=10.0, help="Update rate in Hz")
    parser.add_argument("--speed", type=int, default=1500, help="Servo speed for balance adjustments")
    parser.add_argument("--acc", type=int, default=30, help="Servo acceleration for balance adjustments")
    parser.add_argument("--log-data", action="store_true", help="Log IMU and adjustment data")
    parser.add_argument("--debug-servos", action="store_true", help="Show detailed servo position changes")
    parser.add_argument("--test-mode", action="store_true", help="Test mode with larger adjustments for visibility")
    parser.add_argument("--test-servos", action="store_true", help="Test servo movement before starting balance control")
    parser.add_argument("--command-delay", type=float, default=0.05, help="Delay between servo commands in seconds (default 0.05)")
    args = parser.parse_args()

    # Setup servo manager
    servo_manager = ServoManager()
    ports = servo_manager.list_serial_ports()
    servo_port = args.servo_port or (ports[0] if ports else None)
    if not servo_port:
        print("No servo serial ports found. Specify --servo-port.")
        return 2
    
    ok, err = servo_manager.connect(servo_port, args.servo_baud)
    if not ok:
        print(f"Servo connect failed: {err}")
        return 2

    # Setup MSP serial
    try:
        msp = serial.Serial(args.msp_port, args.msp_baud, timeout=0.1)
        print(f"Connected to MSP port {args.msp_port} at {args.msp_baud} baud")
    except Exception as e:
        print(f"Failed to open MSP port {args.msp_port}: {e}")
        servo_manager.disconnect()
        return 2

    # Setup balance controller
    mgr_lock = threading.Lock()
    balance_controller = BalanceController(servo_manager, mgr_lock)
    
    # Configure balance parameters
    balance_controller.base_height_mm = args.base_height
    balance_controller.max_adjustment_mm = args.max_adjustment
    balance_controller.sensitivity = args.sensitivity
    balance_controller.deadband_deg = args.deadband
    balance_controller.speed = args.speed
    balance_controller.acc = args.acc
    balance_controller.command_delay = args.command_delay
    
    # Test mode: increase sensitivity and reduce deadband for more visible movements
    if args.test_mode:
        balance_controller.sensitivity = 2.0  # Much more aggressive
        balance_controller.deadband_deg = 0.1  # Almost no deadband
        balance_controller.max_adjustment_mm = 25.0  # Larger max adjustments
        print("TEST MODE: Increased sensitivity and reduced deadband for visible movements")

    # Initialize robot to base height
    print(f"Setting robot to base height: {args.base_height}mm")
    if not balance_controller.set_base_height(args.base_height):
        print("Failed to set initial height")
        servo_manager.disconnect()
        msp.close()
        return 2
    
    time.sleep(2.0)  # Wait longer for servos to reach position

    # Test servo movement if requested
    if args.test_servos:
        if not balance_controller.test_servo_movement():
            print("Servo test failed - check connections and servo IDs")
            servo_manager.disconnect()
            msp.close()
            return 2

    rate_dt = 1.0 / max(1.0, args.rate)
    
    print("Balance control started. Reading IMU data and adjusting leg heights...")
    print("Press Ctrl+C to exit")
    print("-" * 60)
    
    try:
        while True:
            start_time = time.time()
            
            # Read IMU attitude data
            try:
                msp.reset_input_buffer()
                pkt = build_msp_request(MSP_ATTITUDE)
                msp.write(pkt)
                time.sleep(0.01)
                resp = read_msp_response(msp)
                attitude_data = parse_msp_attitude(resp)
                
                if attitude_data:
                    roll_raw, pitch_raw, yaw_raw = attitude_data
                    roll_deg = roll_raw / 10.0
                    pitch_deg = pitch_raw / 10.0
                    yaw_deg = yaw_raw / 10.0
                    
                    # Apply balance correction
                    success = balance_controller.apply_balance_correction(roll_deg, pitch_deg, debug=args.debug_servos)
                    
                    if args.log_data or args.debug_servos:
                        adjustments = balance_controller.calculate_leg_adjustments(roll_deg, pitch_deg)
                        print(f"Roll: {roll_deg:6.1f}°, Pitch: {pitch_deg:6.1f}°, Yaw: {yaw_deg:6.1f}°")
                        print(f"Adjustments: FL:{adjustments['FL']:+5.1f}mm FR:{adjustments['FR']:+5.1f}mm "
                              f"RL:{adjustments['RL']:+5.1f}mm RR:{adjustments['RR']:+5.1f}mm")
                        print(f"Success: {success}")
                        if args.debug_servos:
                            print("Servo position changes:")
                        print("-" * 40)
                else:
                    if args.log_data:
                        print("Failed to read attitude data")
                        
            except Exception as e:
                if args.log_data:
                    print(f"Error reading IMU: {e}")
            
            # Maintain update rate
            elapsed = time.time() - start_time
            sleep_time = max(0, rate_dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        try:
            # Return to neutral position
            print("Returning to neutral position...")
            balance_controller.set_base_height(args.base_height)
            time.sleep(1.0)
        except Exception:
            pass
        try:
            msp.close()
        except Exception:
            pass
        servo_manager.disconnect()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
