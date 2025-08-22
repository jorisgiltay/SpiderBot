#!/usr/bin/env python
import argparse
import json
import math
import os
import sys
import time
from typing import Dict, Tuple


# Allow importing from webInterface
CURRENT_DIR = os.path.dirname(__file__)
WEB_IFACE_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "..", "webInterface"))
if WEB_IFACE_DIR not in sys.path:
    sys.path.append(WEB_IFACE_DIR)

from servo_manager import ServoManager  # type: ignore


def load_servo_ranges() -> Dict:
    ranges_path = os.path.abspath(os.path.join(CURRENT_DIR, "..", "servo_ranges.json"))
    with open(ranges_path, "r", encoding="utf-8") as f:
        return json.load(f)


def clamp_ticks(val: int, smin: int, smax: int) -> int:
    return max(smin, min(smax, val))


def map_height_to_knee_ticks(height_mm: float, base_mid: int, min_ticks: int, max_ticks: int) -> int:
    # 0 mm -> max_ticks (fully bent, lowest)
    # 60 mm -> base_mid (highest practical per provided data)
    max_height_mm = 60.0
    h = max(0.0, min(max_height_mm, float(height_mm)))
    t = h / max_height_mm
    target = int(round((1.0 - t) * max_ticks + t * base_mid))
    return clamp_ticks(target, min_ticks, max_ticks)


def hip_offset_for_step(step_mm: float, hip_range_ticks: int, hip_mm_range: float) -> int:
    # Convert linear step in mm to ticks using a simple linear approximation.
    # We approximate hip travel of +/- hip_mm_range mm maps to +/- hip_range_ticks/2 ticks.
    if hip_mm_range <= 0.0:
        return 0
    ticks_per_mm = (hip_range_ticks / 2.0) / hip_mm_range
    return int(round(step_mm * ticks_per_mm))


def main() -> int:
    parser = argparse.ArgumentParser(description="Diagonal gait forward walker at low height (~20mm)")
    parser.add_argument("--port", required=False, help="Serial port (e.g. COM3). If omitted, first detected is used.")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Baudrate (default: 1_000_000)")
    parser.add_argument("--speed", type=int, default=2400, help="Servo speed (ticks/s)")
    parser.add_argument("--acc", type=int, default=50, help="Servo acceleration")
    parser.add_argument("--height", type=float, default=20.0, help="Body height in mm (target ~20)")
    parser.add_argument("--lift", type=float, default=8.0, help="Extra knee height added during swing (mm)")
    parser.add_argument("--step", type=float, default=18.0, help="Step length in mm at the hip arc")
    parser.add_argument("--stride-time", type=float, default=0.8, help="Time (s) for a full cycle (two swings)")
    parser.add_argument("--cycles", type=int, default=0, help="Number of full cycles to walk (<=0 for infinite)")
    parser.add_argument("--hip-mm-range", type=float, default=35.0, help="Approx hip linear range per side in mm for mapping")
    parser.add_argument("--hip-sign-left", type=int, default=1, choices=[-1, 1], help="Sign to apply for left hips (+1 or -1)")
    parser.add_argument("--hip-sign-right", type=int, default=1, choices=[-1, 1], help="Sign to apply for right hips (+1 or -1)")
    parser.add_argument("--hold-after", type=float, default=0.5, help="Hold seconds after finishing")
    args = parser.parse_args()

    # Servo mapping per user's description
    # FL: hip 1, knee 2 | FR: hip 3, knee 4 | RL: hip 5, knee 6 | RR: hip 7, knee 8
    FL = (1, 2)
    FR = (3, 4)
    RL = (5, 6)
    RR = (7, 8)

    # Diagonal pairs for trot: FL+RR and FR+RL
    pair_A = (FL, RR)
    pair_B = (FR, RL)

    ranges = load_servo_ranges()["servos"]

    mgr = ServoManager()
    ports = mgr.list_serial_ports()
    port_to_use = args.port or (ports[0] if ports else None)
    if not port_to_use:
        print("No serial ports found. Specify --port.")
        return 2

    ok, err = mgr.connect(port_to_use, args.baud)
    if not ok:
        print(f"Connect failed: {err}")
        return 2

    def servo_mid(sid: int) -> int:
        s = ranges[f"servo{sid}"]
        return clamp_ticks(int(s["mid"]), int(s["min"]), int(s["max"]))

    def knee_for_height(sid: int, h_mm: float) -> int:
        s = ranges[f"servo{sid}"]
        return map_height_to_knee_ticks(h_mm, int(s["mid"]), int(s["min"]), int(s["max"]))

    def hip_bounds(sid: int) -> Tuple[int, int, int]:
        s = ranges[f"servo{sid}"]
        return int(s["min"]), int(s["mid"]), int(s["max"])  # (min, mid, max)

    # Precompute neutral pose at requested height
    neutral = {
        1: servo_mid(1),
        2: knee_for_height(2, args.height),
        3: servo_mid(3),
        4: knee_for_height(4, args.height),
        5: servo_mid(5),
        6: knee_for_height(6, args.height),
        7: servo_mid(7),
        8: knee_for_height(8, args.height),
    }

    # Convert step length to hip tick offset around neutral
    # We use each hip's full range to estimate scaling
    hip_tick_half_ranges: Dict[int, int] = {}
    for hip_id in (1, 3, 5, 7):
        hmin, hmid, hmax = hip_bounds(hip_id)
        hip_tick_half_ranges[hip_id] = int((hmax - hmin) / 2)

    hip_step_ticks: Dict[int, int] = {}
    for hip_id in (1, 3, 5, 7):
        hip_step_ticks[hip_id] = hip_offset_for_step(args.step, hip_tick_half_ranges[hip_id] * 2, args.hip_mm_range)

    # Timing
    full_T = max(0.2, float(args.stride_time))
    step_T = full_T / 4.0  # four swings per full cycle

    try:
        # Go to neutral pose first
        for sid, pos in neutral.items():
            ok, err = mgr.write_position(sid, pos, args.speed, args.acc)
            if not ok:
                print(f"Init write failed on {sid}: {err}")
        time.sleep(0.5)

        def set_leg(hip_id: int, knee_id: int, hip_pos: int, knee_pos: int) -> None:
            ok, err = mgr.write_position(hip_id, hip_pos, args.speed, args.acc)
            if not ok:
                print(f"Hip {hip_id} write failed: {err}")
            ok, err = mgr.write_position(knee_id, knee_pos, args.speed, args.acc)
            if not ok:
                print(f"Knee {knee_id} write failed: {err}")

        def side_sign_for_hip(hip_id: int) -> int:
            # Always invert right hips per request (right hips are 3 and 7)
            return -1 if hip_id in (3, 7) else 1

        def stance_phase(hip_id: int, knee_id: int, forward: bool) -> None:
            # Hips positive = clockwise. We'll define forward as moving foot backward relative to body during stance
            hmin, hmid, hmax = hip_bounds(hip_id)
            base = hmid
            step = hip_step_ticks[hip_id]
            side_sign = side_sign_for_hip(hip_id)
            signed_step = side_sign * step
            if forward:
                target = clamp_ticks(base - signed_step, hmin, hmax)  # move leg backward relative to body
            else:
                target = clamp_ticks(base + signed_step, hmin, hmax)
            set_leg(hip_id, knee_id, target, knee_for_height(knee_id, args.height))

        def swing_phase(hip_id: int, knee_id: int, forward: bool) -> None:
            # Lift knee (reduce ticks as negative lifts body per description)
            base_knee = knee_for_height(knee_id, args.height)
            lift_knee = knee_for_height(knee_id, max(0.0, args.height - args.lift))
            hmin, hmid, hmax = hip_bounds(hip_id)
            base = hmid
            step = hip_step_ticks[hip_id]
            side_sign = side_sign_for_hip(hip_id)
            signed_step = side_sign * step
            # During swing, move leg forward relative to body
            if forward:
                target = clamp_ticks(base + signed_step, hmin, hmax)
            else:
                target = clamp_ticks(base - signed_step, hmin, hmax)
            set_leg(hip_id, knee_id, target, lift_knee)
            # Brief pause to let swing occur before lowering
            time.sleep(step_T * 0.5)
            # Lower back to baseline height at the swing end
            ok, err = mgr.write_position(knee_id, base_knee, args.speed, args.acc)
            if not ok:
                print(f"Knee {knee_id} write failed: {err}")
            time.sleep(max(0.0, step_T * 0.5))

        # Execute cycles or loop infinitely; sweep one leg at a time in diagonal order
        try:
            leg_sequence = [FL, RR, FR, RL]
            if int(args.cycles) <= 0:
                while True:
                    for swing_hip, swing_knee in leg_sequence:
                        # All other legs take stance
                        for (hip_id, knee_id) in [FL, RR, FR, RL]:
                            if hip_id != swing_hip:
                                stance_phase(hip_id, knee_id, forward=True)
                        # Swing selected leg
                        swing_phase(swing_hip, swing_knee, forward=True)
            else:
                for _ in range(int(args.cycles)):
                    for swing_hip, swing_knee in leg_sequence:
                        for (hip_id, knee_id) in [FL, RR, FR, RL]:
                            if hip_id != swing_hip:
                                stance_phase(hip_id, knee_id, forward=True)
                        swing_phase(swing_hip, swing_knee, forward=True)
        except KeyboardInterrupt:
            pass

        # Return to neutral
        for sid, pos in neutral.items():
            ok, err = mgr.write_position(sid, pos, args.speed, args.acc)
            if not ok:
                print(f"Return write failed on {sid}: {err}")
        time.sleep(max(0.0, float(args.hold_after)))
        return 0
    finally:
        try:
            # Attempt to return to neutral safely on exit/interrupt
            for sid, pos in neutral.items():
                try:
                    mgr.write_position(sid, pos, 2000, args.acc)
                except Exception:
                    pass
            time.sleep(0.3)
        finally:
            mgr.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())


