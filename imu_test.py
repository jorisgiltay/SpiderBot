#!/usr/bin/env python
"""
Test script to read IMU data from Betaflight flight controller over MSP.

This script reads raw IMU data (accelerometer, gyroscope, magnetometer) and
attitude data (roll, pitch, yaw) from a Betaflight flight controller using
the MultiWii Serial Protocol (MSP).

Usage:
    python imu_test.py --port COM4 --baud 115200
    python imu_test.py --port /dev/ttyAMA0 --baud 115200
"""

import argparse
import time
import struct
from typing import Optional, Tuple

import serial


# ---------- MSP Protocol Constants ----------
MSP_RAW_IMU = 102    # Raw IMU data (accel, gyro, mag)
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


def parse_msp_raw_imu(response: bytes) -> Optional[Tuple[int, int, int, int, int, int, int, int, int]]:
    """
    Parse MSP_RAW_IMU response.
    Returns: (ax, ay, az, gx, gy, gz, mx, my, mz) or None if invalid
    """
    if len(response) < 5:
        return None
    
    size = response[3]
    if size != 18:  # 9 * 2 bytes per int16
        return None
    
    payload = response[5:5 + size]
    if len(payload) != 18:
        return None
    
    # Unpack 9 signed 16-bit integers (little-endian)
    data = struct.unpack('<9h', payload)
    return data


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


# ---------- Data Formatting ----------
def format_imu_data(imu_data: Tuple[int, int, int, int, int, int, int, int, int]) -> str:
    """Format IMU data for display."""
    ax, ay, az, gx, gy, gz, mx, my, mz = imu_data
    
    # Convert to more readable units
    # Note: These are raw values from the flight controller
    # The actual scaling depends on your FC's configuration
    accel_scale = 1.0  # Adjust based on your FC's accel scale
    gyro_scale = 1.0   # Adjust based on your FC's gyro scale
    mag_scale = 1.0    # Adjust based on your FC's mag scale
    
    return f"""
Raw IMU Data:
  Accelerometer: X={ax:6d}, Y={ay:6d}, Z={az:6d}
  Gyroscope:     X={gx:6d}, Y={gy:6d}, Z={gz:6d}
  Magnetometer:  X={mx:6d}, Y={my:6d}, Z={mz:6d}
"""


def format_attitude_data(attitude_data: Tuple[int, int, int]) -> str:
    """Format attitude data for display."""
    roll, pitch, yaw = attitude_data
    
    # Convert from 1/10 degree units to degrees
    roll_deg = roll / 10.0
    pitch_deg = pitch / 10.0
    yaw_deg = yaw / 10.0
    
    return f"""
Attitude Data:
  Roll:  {roll_deg:7.1f}° (raw: {roll:6d})
  Pitch: {pitch_deg:7.1f}° (raw: {pitch:6d})
  Yaw:   {yaw_deg:7.1f}° (raw: {yaw:6d})
"""


# ---------- Main Function ----------
def main() -> int:
    parser = argparse.ArgumentParser(description="Read IMU data from Betaflight flight controller over MSP")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port (e.g. COM4 or /dev/ttyAMA0)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    parser.add_argument("--rate", type=float, default=10.0, help="Update rate in Hz (default 10.0)")
    parser.add_argument("--raw-only", action="store_true", help="Only show raw IMU data (skip attitude)")
    parser.add_argument("--attitude-only", action="store_true", help="Only show attitude data (skip raw IMU)")
    args = parser.parse_args()

    # Setup serial connection
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        print(f"Connected to {args.port} at {args.baud} baud")
    except Exception as e:
        print(f"Failed to open serial port {args.port}: {e}")
        return 1

    rate_dt = 1.0 / max(1.0, args.rate)
    
    print("Reading IMU data from flight controller...")
    print("Press Ctrl+C to exit")
    print("-" * 50)
    
    try:
        while True:
            start_time = time.time()
            
            # Request raw IMU data
            if not args.attitude_only:
                try:
                    ser.reset_input_buffer()
                    pkt = build_msp_request(MSP_RAW_IMU)
                    ser.write(pkt)
                    time.sleep(0.01)
                    resp = read_msp_response(ser)
                    imu_data = parse_msp_raw_imu(resp)
                    
                    if imu_data:
                        print(format_imu_data(imu_data))
                    else:
                        print("Failed to read raw IMU data")
                except Exception as e:
                    print(f"Error reading raw IMU: {e}")
            
            # Request attitude data
            if not args.raw_only:
                try:
                    ser.reset_input_buffer()
                    pkt = build_msp_request(MSP_ATTITUDE)
                    ser.write(pkt)
                    time.sleep(0.01)
                    resp = read_msp_response(ser)
                    attitude_data = parse_msp_attitude(resp)
                    
                    if attitude_data:
                        print(format_attitude_data(attitude_data))
                    else:
                        print("Failed to read attitude data")
                except Exception as e:
                    print(f"Error reading attitude: {e}")
            
            print("-" * 50)
            
            # Maintain update rate
            elapsed = time.time() - start_time
            sleep_time = max(0, rate_dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        try:
            ser.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
