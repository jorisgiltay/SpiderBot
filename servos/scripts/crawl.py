#!/usr/bin/env python
import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass


# Allow importing from webInterface
CURRENT_DIR = os.path.dirname(__file__)
WEB_IFACE_DIR = os.path.abspath(os.path.join(CURRENT_DIR, "..", "webInterface"))
if WEB_IFACE_DIR not in sys.path:
    sys.path.append(WEB_IFACE_DIR)

from servo_manager import ServoManager  # type: ignore


def load_servo_ranges() -> dict:
    ranges_path = os.path.abspath(os.path.join(CURRENT_DIR, "..", "servo_ranges.json"))
    with open(ranges_path, "r", encoding="utf-8") as f:
        return json.load(f)


def clamp_ticks(val: int, smin: int, smax: int) -> int:
    return max(smin, min(smax, val))


@dataclass
class LegIds:
    hip: int
    knee: int


def plan_crawl_sequence() -> list:
    # Classic crawl: always 3 contacts. Sequence example (lift/move hip+knee together):
    # 1) Lift and advance front-left (1,2)
    # 2) Lift and advance rear-right (7,8)
    # 3) Lift and advance front-right (3,4)
    # 4) Lift and advance rear-left (5,6)
    return [
        LegIds(hip=1, knee=2),
        LegIds(hip=7, knee=8),
        LegIds(hip=3, knee=4),
        LegIds(hip=5, knee=6),
    ]


# Direction inversion configuration
# Invert knee direction for all knees; invert hip direction for hips 3 and 7
HIP_INVERT_IDS = {3, 7}
KNEE_INVERT_IDS = {2, 4, 6, 8}


def safe_write_position(manager: ServoManager, sid: int, pos: int, speed: int, acc: int, retries: int = 3, pause_s: float = 0.05) -> bool:
    # Ensure torque, try multiple times with small pauses
    ok, _ = manager.set_torque(sid, True)
    last_err: str = ""
    for _ in range(max(1, int(retries))):
        ok, err = manager.write_position(sid, pos, speed, acc)
        if ok:
            return True
        last_err = err or ""
        time.sleep(max(0.0, float(pause_s)))
        # try re-enable torque again before retry
        manager.set_torque(sid, True)
    if last_err:
        print(f"Write position {sid} failed after retries: {last_err}")
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description="Scaffold crawl gait maintaining 3 contact points.")
    parser.add_argument("--port", required=False, help="Serial port (e.g. COM3). If omitted, first detected is used.")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Baudrate (default: 1_000_000)")
    parser.add_argument("--speed", type=int, default=1200, help="Servo speed (ticks/s)")
    parser.add_argument("--acc", type=int, default=20, help="Servo acceleration")
    parser.add_argument("--step", type=int, default=120, help="Hip delta ticks per step forward")
    parser.add_argument("--lift", type=int, default=120, help="Knee lift ticks for swing phase")
    parser.add_argument("--cycles", type=int, default=1, help="Number of full crawl cycles")
    parser.add_argument("--pause", type=float, default=0.25, help="Pause between sub-steps (s)")
    parser.add_argument("--nudges", action="store_true", help="Enable small hip nudges for CoG (off by default)")
    args = parser.parse_args()

    ranges = load_servo_ranges()["servos"]
    hip_ids = [1, 3, 5, 7]
    knee_ids = [2, 4, 6, 8]

    manager = ServoManager()
    ports = manager.list_serial_ports()
    port_to_use = args.port or (ports[0] if ports else None)
    if not port_to_use:
        print("No serial ports found. Specify --port.")
        return 2

    ok, err = manager.connect(port_to_use, args.baud)
    if not ok:
        print(f"Connect failed: {err}")
        return 2

    try:
        # Enable torque on all involved servos upfront
        for sid in hip_ids + knee_ids:
            manager.set_torque(sid, True)
        time.sleep(0.05)

        # Start from neutral stand
        for sid in hip_ids + knee_ids:
            s = ranges[f"servo{sid}"]
            pos = clamp_ticks(int(s["mid"]), int(s["min"]), int(s["max"]))
            ok = safe_write_position(manager, sid, pos, args.speed, args.acc)
            if not ok:
                print(f"Init {sid} write failed")
        time.sleep(0.5)

        sequence = plan_crawl_sequence()
        for _ in range(max(1, int(args.cycles))):
            for leg in sequence:
                hip = leg.hip
                knee = leg.knee

                # Lift swing leg
                s_knee = ranges[f"servo{knee}"]
                if knee in KNEE_INVERT_IDS:
                    knee_lift = clamp_ticks(int(s_knee["mid"]) + int(args.lift), int(s_knee["min"]), int(s_knee["max"]))
                else:
                    knee_lift = clamp_ticks(int(s_knee["mid"]) - int(args.lift), int(s_knee["min"]), int(s_knee["max"]))
                ok = safe_write_position(manager, knee, knee_lift, args.speed, args.acc)
                if not ok:
                    print(f"Knee lift {knee} failed")
                time.sleep(args.pause)

                # Advance hip forward
                s_hip = ranges[f"servo{hip}"]
                hip_delta = int(args.step)
                if hip in HIP_INVERT_IDS:
                    hip_delta = -hip_delta
                hip_forward = clamp_ticks(int(s_hip["mid"]) + hip_delta, int(s_hip["min"]), int(s_hip["max"]))
                ok = safe_write_position(manager, hip, hip_forward, args.speed, args.acc)
                if not ok:
                    print(f"Hip advance {hip} failed")
                time.sleep(args.pause)

                # Lower knee to neutral contact
                ok = safe_write_position(manager, knee, int(s_knee["mid"]), args.speed, args.acc)
                if not ok:
                    print(f"Knee lower {knee} failed")
                time.sleep(args.pause)

                # While swing leg moves, consider slight counter-motions on stance hips for CoG centering
                # Minimal scaffold: nudge the two adjacent hips slightly backward, opposite diagonal slightly forward.
                # This is a simple placeholder; tune per-mechanics.
                if args.nudges:
                    def nudge(sid: int, delta: int) -> None:
                        s = ranges[f"servo{sid}"]
                        # Respect hip inversion for nudges as well
                        adj = -delta if sid in HIP_INVERT_IDS else delta
                        tgt = clamp_ticks(int(s["mid"]) + adj, int(s["min"]), int(s["max"]))
                        ok2 = safe_write_position(manager, sid, tgt, args.speed, args.acc)
                        if not ok2:
                            print(f"Nudge {sid} failed")

                    if leg == LegIds(1, 2):
                        nudge(3, -args.step // 6)
                        nudge(5, -args.step // 6)
                        nudge(7, +args.step // 8)
                    elif leg == LegIds(7, 8):
                        nudge(5, -args.step // 6)
                        nudge(3, -args.step // 6)
                        nudge(1, +args.step // 8)
                    elif leg == LegIds(3, 4):
                        nudge(1, -args.step // 6)
                        nudge(7, -args.step // 6)
                        nudge(5, +args.step // 8)
                    elif leg == LegIds(5, 6):
                        nudge(7, -args.step // 6)
                        nudge(1, -args.step // 6)
                        nudge(3, +args.step // 8)

                time.sleep(args.pause)

        # Return to neutral
        for sid in hip_ids + knee_ids:
            s = ranges[f"servo{sid}"]
            ok = safe_write_position(manager, sid, int(s["mid"]), args.speed, args.acc)
            if not ok:
                print(f"Reset {sid} failed")
        time.sleep(0.4)
        return 0
    finally:
        manager.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())


