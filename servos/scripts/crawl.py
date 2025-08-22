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


def deg_to_ticks(deg: float) -> int:
    # 4095 ticks ≈ 360 degrees
    return int(round(float(deg) * 4095.0 / 360.0))


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
    parser.add_argument("--speed", type=int, default=1200, help="Servo speed (ticks/s). Use --speed-deg to set in deg/s")
    parser.add_argument("--speed-deg", type=float, default=None, help="Servo speed in deg/s (converted to ticks/s)")
    parser.add_argument("--acc", type=int, default=20, help="Servo acceleration (ticks unit)")
    parser.add_argument("--step", type=float, default=16.0, help="Hip sweep in degrees per step")
    parser.add_argument("--lift", type=float, default=16.0, help="Knee lift in degrees for swing phase")
    parser.add_argument("--cycles", type=int, default=1, help="Number of full crawl cycles")
    parser.add_argument("--pause", type=float, default=0.25, help="Pause between sub-steps (s)")
    parser.add_argument("--nudges", action="store_true", help="Enable small hip nudges for CoG (off by default)")
    parser.add_argument("--stance-bend", type=float, default=9.0, help="Extra knee bend in degrees during stance to keep CoG low")
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
        # Determine speed in ticks/s if --speed-deg provided
        speed_ticks = int(args.speed)
        if args.speed_deg is not None:
            speed_ticks = deg_to_ticks(float(args.speed_deg))

        # Precompute deltas in ticks
        step_ticks = max(1, deg_to_ticks(float(args.step)))
        lift_ticks = max(1, deg_to_ticks(float(args.lift)))
        stance_bend_ticks = max(0, deg_to_ticks(float(args.stance_bend)))

        # Enable torque on all involved servos upfront
        for sid in hip_ids + knee_ids:
            manager.set_torque(sid, True)
        time.sleep(0.05)

        # Start from neutral stand (hips at mid, knees bent for lower CoG)
        for sid in hip_ids + knee_ids:
            s = ranges[f"servo{sid}"]
            if sid in knee_ids:
                bend_delta = stance_bend_ticks if sid in KNEE_INVERT_IDS else -stance_bend_ticks
                pos = clamp_ticks(int(s["mid"]) + bend_delta, int(s["min"]), int(s["max"]))
            else:
                pos = clamp_ticks(int(s["mid"]), int(s["min"]), int(s["max"]))
            ok = safe_write_position(manager, sid, pos, speed_ticks, args.acc)
            if not ok:
                print(f"Init {sid} write failed")
        time.sleep(0.5)

        sequence = plan_crawl_sequence()
        for _ in range(max(1, int(args.cycles))):
            for leg in sequence:
                hip = leg.hip
                knee = leg.knee

                # Pre-position: sweep hip backward while foot is on ground to prepare forward swing
                s_hip = ranges[f"servo{hip}"]
                hip_delta = step_ticks
                if hip in HIP_INVERT_IDS:
                    hip_delta = -hip_delta
                hip_back = clamp_ticks(int(s_hip["mid"]) - hip_delta, int(s_hip["min"]), int(s_hip["max"]))
                ok = safe_write_position(manager, hip, hip_back, speed_ticks, args.acc)
                if not ok:
                    print(f"Hip preload back {hip} failed")
                time.sleep(args.pause)

                # Lift swing leg
                s_knee = ranges[f"servo{knee}"]
                if knee in KNEE_INVERT_IDS:
                    knee_lift = clamp_ticks(int(s_knee["mid"]) + lift_ticks, int(s_knee["min"]), int(s_knee["max"]))
                else:
                    knee_lift = clamp_ticks(int(s_knee["mid"]) - lift_ticks, int(s_knee["min"]), int(s_knee["max"]))
                ok = safe_write_position(manager, knee, knee_lift, speed_ticks, args.acc)
                if not ok:
                    print(f"Knee lift {knee} failed")
                time.sleep(args.pause)

                # Advance hip forward
                hip_forward = clamp_ticks(int(s_hip["mid"]) + hip_delta, int(s_hip["min"]), int(s_hip["max"]))
                ok = safe_write_position(manager, hip, hip_forward, speed_ticks, args.acc)
                if not ok:
                    print(f"Hip advance {hip} failed")
                time.sleep(args.pause)

                # Lower knee to stance contact (keep CoG low)
                bend_delta = stance_bend_ticks if knee in KNEE_INVERT_IDS else -stance_bend_ticks
                knee_contact = clamp_ticks(int(s_knee["mid"]) + bend_delta, int(s_knee["min"]), int(s_knee["max"]))
                ok = safe_write_position(manager, knee, knee_contact, speed_ticks, args.acc)
                if not ok:
                    print(f"Knee lower {knee} failed")
                time.sleep(args.pause)

                # While swing leg moves, consider slight counter-motions on stance hips for CoG centering
                # Minimal scaffold: nudge the two adjacent hips slightly backward, opposite diagonal slightly forward.
                # This is a simple placeholder; tune per-mechanics.
                if args.nudges:
                    def nudge(sid: int, delta_deg: float) -> None:
                        s = ranges[f"servo{sid}"]
                        # Convert degrees to ticks and respect hip inversion
                        base_ticks = max(1, deg_to_ticks(float(delta_deg)))
                        adj = -base_ticks if sid in HIP_INVERT_IDS else base_ticks
                        tgt = clamp_ticks(int(s["mid"]) + adj, int(s["min"]), int(s["max"]))
                        ok2 = safe_write_position(manager, sid, tgt, speed_ticks, args.acc)
                        if not ok2:
                            print(f"Nudge {sid} failed")

                    if leg == LegIds(1, 2):
                        nudge(3, -args.step / 6.0)
                        nudge(5, -args.step / 6.0)
                        nudge(7, +args.step / 8.0)
                    elif leg == LegIds(7, 8):
                        nudge(5, -args.step / 6.0)
                        nudge(3, -args.step / 6.0)
                        nudge(1, +args.step / 8.0)
                    elif leg == LegIds(3, 4):
                        nudge(1, -args.step / 6.0)
                        nudge(7, -args.step / 6.0)
                        nudge(5, +args.step / 8.0)
                    elif leg == LegIds(5, 6):
                        nudge(7, -args.step / 6.0)
                        nudge(1, -args.step / 6.0)
                        nudge(3, +args.step / 8.0)

                time.sleep(args.pause)

        # Return to stance (hips mid, knees bent)
        for sid in hip_ids + knee_ids:
            s = ranges[f"servo{sid}"]
            if sid in knee_ids:
                bend_delta = stance_bend_ticks if sid in KNEE_INVERT_IDS else -stance_bend_ticks
                pos = clamp_ticks(int(s["mid"]) + bend_delta, int(s["min"]), int(s["max"]))
            else:
                pos = clamp_ticks(int(s["mid"]), int(s["min"]), int(s["max"]))
            ok = safe_write_position(manager, sid, pos, speed_ticks, args.acc)
            if not ok:
                print(f"Reset {sid} failed")
        time.sleep(0.4)
        return 0
    finally:
        manager.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())


