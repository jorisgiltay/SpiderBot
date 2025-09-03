#!/usr/bin/env python
import argparse
import os
import sys
import time
import struct
from typing import List, Optional, Tuple

import serial


# ---------- MSP (RC) protocol helpers ----------
MSP_RC = 105  # RC stick/channel data


def build_msp_request(command: int, payload: bytes = b"") -> bytes:
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
    # Read until we find a full MSP frame ('$M>' + size + cmd + payload + checksum)
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


def parse_msp_rc(response: bytes) -> List[int]:
    # Match poll_stick.py: extract size at [3], payload starts at [5]
    if len(response) < 5:
        return []
    size = response[3]
    payload = response[5 : 5 + size]
    channels: List[int] = []
    for i in range(0, len(payload), 2):
        if i + 2 <= len(payload):
            ch = struct.unpack("<H", payload[i : i + 2])[0]
            channels.append(ch)
    return channels


# ---------- Servo control imports ----------
# Ensure we can import from servos/webInterface
HERE = os.path.dirname(__file__)
WEB_IFACE_DIR = os.path.abspath(os.path.join(HERE, "servos", "webInterface"))
if WEB_IFACE_DIR not in sys.path:
    sys.path.append(WEB_IFACE_DIR)

from motion_controller import MotionController  # type: ignore
from servo_manager import ServoManager  # type: ignore
import threading


# ---------- Mapping utilities ----------
def normalize_axis(val: int, deadband: float = 0.12) -> float:
    """
    Map RC pulse [990..2000] to [-1..+1] with deadband around center (~1500).
    """
    # Clamp first
    v = max(990, min(2000, int(val)))
    # Scale to -1..1 using center 1500 and half-span ~500
    norm = (v - 1500.0) / 500.0
    # Deadband
    if abs(norm) < deadband:
        return 0.0
    # Clamp
    if norm > 1.0:
        norm = 1.0
    if norm < -1.0:
        norm = -1.0
    return norm


def map_height_mm(val: int, min_mm: float = 0.0, max_mm: float = 60.0) -> float:
    """
    Map RC pulse [990..2000] to height in mm [min_mm..max_mm].
    """
    v = max(990, min(2000, int(val)))
    t = (v - 990.0) / (2000.0 - 990.0)
    return float(min_mm + t * (max_mm - min_mm))


class RCIntent:
    def __init__(self, active: bool, mode: Optional[str], height_mm: float):
        self.active = active
        self.mode = mode
        self.height_mm = height_mm


class RCInterpreter:
    """
    Interpret MSP RC channels into gait mode, height and activation.

    Channel indices (0-based):
      6: crab left/right (left < 0, right > 0)
      7: forward/backward (forward > 0, backward < 0)
      8: pivot (pivot_right > 0, pivot_left < 0)
      9: height (0..60mm)
     10: activation (on/off with hysteresis)
    """

    def __init__(self, on_threshold: int = 1600, off_threshold: int = 1400, deadband: float = 0.12):
        self.on_threshold = on_threshold
        self.off_threshold = off_threshold
        self.deadband = deadband
        self._active_state = False

    def _activation(self, val: int) -> bool:
        if self._active_state:
            if val <= self.off_threshold:
                self._active_state = False
        else:
            if val >= self.on_threshold:
                self._active_state = True
        return self._active_state

    def interpret(self, channels: List[int]) -> Optional[RCIntent]:
        if len(channels) <= 10:
            return None

        strafe_axis = normalize_axis(channels[6], self.deadband)
        fwd_axis = normalize_axis(channels[7], self.deadband)
        pivot_axis = normalize_axis(channels[8], self.deadband)
        height = map_height_mm(channels[9])
        active = self._activation(channels[10])

        # Choose dominant axis if active
        mode: Optional[str] = None
        if active:
            # Compute magnitudes
            mags: List[Tuple[str, float]] = [
                ("pivot_right" if pivot_axis > 0 else "pivot_left", abs(pivot_axis)),
                ("forward" if fwd_axis > 0 else "backward", abs(fwd_axis)),
                ("right" if strafe_axis > 0 else "left", abs(strafe_axis)),
            ]
            # Pick the direction with the largest magnitude, but only if above deadband (> 0 due to normalize)
            best = max(mags, key=lambda kv: kv[1])
            if best[1] > 0.0:
                mode = best[0]

        return RCIntent(active=active, mode=mode, height_mm=height)


# ---------- Main control loop ----------
def main() -> int:
    parser = argparse.ArgumentParser(description="Poll MSP RC and drive servos (gait/height/activation)")
    parser.add_argument("--msp-port", default="/dev/ttyAMA0", help="MSP/FC serial port (e.g. COM4 or /dev/ttyAMA0)")
    parser.add_argument("--msp-baud", type=int, default=115200, help="MSP/FC baud rate (default 115200)")
    parser.add_argument("--servo-port", default=None, help="Servo bus serial port (if omitted, first detected is used)")
    parser.add_argument("--servo-baud", type=int, default=1_000_000, help="Servo bus baud (default 1_000_000)")
    parser.add_argument("--speed", type=int, default=3000, help="Servo speed for gaits")
    parser.add_argument("--acc", type=int, default=90, help="Servo acceleration for gaits")
    parser.add_argument("--lift", type=float, default=10.0, help="Lift height (mm) during swing")
    parser.add_argument("--step", type=float, default=12.0, help="Step length (mm) at hip arc")
    parser.add_argument("--stride-time", type=float, default=1, help="Stride time (s) for a full gait cycle")
    parser.add_argument("--hip-mm-range", type=float, default=35.0, help="Approx hip range (mm) for mapping")
    parser.add_argument("--height-hysteresis", type=float, default=3.0, help="Min delta (mm) to apply new height")
    parser.add_argument("--rc-deadband", type=float, default=0.12, help="Deadband for RC axes (fraction of full)")
    parser.add_argument("--on-threshold", type=int, default=1600, help="Activation ON threshold (ch10)")
    parser.add_argument("--off-threshold", type=int, default=1400, help="Activation OFF threshold (ch10)")
    parser.add_argument("--rate-hz", type=float, default=30.0, help="Polling rate in Hz for MSP RC")
    parser.add_argument("--log-rc", action="store_true", help="Print RC channel values each loop")
    args = parser.parse_args()

    # Setup servo manager and motion controller
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

    mgr_lock = threading.Lock()
    controller = MotionController(servo_manager, mgr_lock)

    # Setup MSP serial
    try:
        msp = serial.Serial(args.msp_port, args.msp_baud, timeout=0.05)
    except Exception as e:
        print(f"Failed to open MSP port {args.msp_port}: {e}")
        servo_manager.disconnect()
        return 2

    interpreter = RCInterpreter(on_threshold=args.on_threshold, off_threshold=args.off_threshold, deadband=args.rc_deadband)

    rate_dt = 1.0 / max(1.0, float(args.rate_hz))
    last_height_cmd: Optional[float] = None
    last_mode: Optional[str] = None
    was_active = False

    print("RC drive started. Waiting for activation (ch10)...")
    try:
        while True:
            try:
                # Flush any pending bytes to avoid mixing old/new frames
                try:
                    msp.reset_input_buffer()
                except Exception:
                    pass
                pkt = build_msp_request(MSP_RC)
                msp.write(pkt)
            except Exception:
                # If write fails, brief pause and retry
                time.sleep(0.05)
                continue

            time.sleep(0.01)
            resp = read_msp_response(msp)
            channels = parse_msp_rc(resp)
            if args.log_rc and channels:
                # Match poll_stick.py style: print the list directly
                print("RC Channels:", channels)
            intent = interpreter.interpret(channels)
            if intent is None:
                time.sleep(rate_dt)
                continue

            # Activation handling
            if not intent.active:
                if was_active:
                    controller.stop()
                    last_mode = None
                was_active = False
                # We still allow height adjustments while inactive (stand still at requested height)
                if last_height_cmd is None or abs(intent.height_mm - last_height_cmd) >= float(args.height_hysteresis):
                    controller.set_height(intent.height_mm, speed=args.speed, acc=args.acc)
                    last_height_cmd = intent.height_mm
                time.sleep(rate_dt)
                continue

            # Active
            was_active = True

            # Height management: if gait is running and height changed, restart gait with new params
            if last_height_cmd is None or abs(intent.height_mm - last_height_cmd) >= float(args.height_hysteresis):
                # If a gait is running, we will (re)start it below with updated params
                last_height_cmd = intent.height_mm

            # Choose or stop gait depending on mode determination
            desired_mode = intent.mode
            if desired_mode is None:
                # No axis above deadband -> stop gait, hold height
                if controller.is_running():
                    controller.stop()
                    last_mode = None
                # While active but no gait, maintain height
                controller.set_height(intent.height_mm, speed=args.speed, acc=args.acc)
            else:
                # Start or switch gait if changed or height changed
                if desired_mode != last_mode or not controller.is_running():
                    params = {
                        "speed": int(args.speed),
                        "acc": int(args.acc),
                        "height": float(last_height_cmd if last_height_cmd is not None else intent.height_mm),
                        "lift": float(args.lift),
                        "step": float(args.step),
                        "stride_time": float(args.stride_time),
                        "hip_mm_range": float(args.hip_mm_range),
                    }
                    ok, err = controller.start_gait(desired_mode, params)
                    if not ok:
                        # If start fails, ensure stopped
                        controller.stop()
                        last_mode = None
                    else:
                        last_mode = desired_mode
                else:
                    # Same gait still running: if height changed, restart with new height param
                    # to apply updated base height inside gait loop
                    if last_height_cmd is not None:
                        params = {
                            "speed": int(args.speed),
                            "acc": int(args.acc),
                            "height": float(last_height_cmd),
                            "lift": float(args.lift),
                            "step": float(args.step),
                            "stride_time": float(args.stride_time),
                            "hip_mm_range": float(args.hip_mm_range),
                        }
                        controller.stop()
                        time.sleep(0.05)
                        controller.start_gait(desired_mode, params)

            time.sleep(rate_dt)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        try:
            controller.stop()
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


