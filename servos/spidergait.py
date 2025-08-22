#!/usr/bin/env python
import argparse
import os
import sys
import time
from typing import Dict, Tuple, List


# Make webInterface importable
THIS_DIR = os.path.dirname(__file__)
WEB_IF_DIR = os.path.join(THIS_DIR, "webInterface")
if WEB_IF_DIR not in sys.path:
    sys.path.append(WEB_IF_DIR)

from servo_manager import ServoManager  # type: ignore  # noqa: E402
from gait import parse_ranges_file, compute_hip_targets  # type: ignore  # noqa: E402


Leg = Tuple[int, int]  # (hip_id, knee_id)


def build_leg_order(direction: str) -> List[Leg]:
    """
    Crawl order used for phase offsets: FL → FR → RL → RR
    """
    legs: List[Leg] = [
        (1, 2),  # Front Left
        (3, 4),  # Front Right
        (5, 6),  # Rear Left
        (7, 8),  # Rear Right
    ]
    return legs


def compute_hip_low_high(ranges: Dict[int, Dict[str, int]]) -> Dict[int, Tuple[int, int]]:
    """Compute ±(range/4) around mid, clamped to [min, max] for each hip id."""
    result: Dict[int, Tuple[int, int]] = {}
    for hip_id in (1, 3, 5, 7):
        r = ranges.get(hip_id, {})
        if not {"min", "mid", "max"}.issubset(r.keys()):
            raise ValueError(f"Missing min/mid/max for hip servo {hip_id}")
        low, high = compute_hip_targets(int(r["min"]), int(r["mid"]), int(r["max"]))
        result[hip_id] = (low, high)
    return result


def ensure_all_present(ranges: Dict[int, Dict[str, int]], ids: List[int]) -> None:
    for sid in ids:
        r = ranges.get(sid)
        if r is None:
            raise ValueError(f"Missing ranges for servo {sid}")
        for k in ("min", "mid", "max"):
            if k not in r:
                raise ValueError(f"Missing '{k}' for servo {sid}")


def lerp(a: float, b: float, u: float) -> float:
    return a + (b - a) * max(0.0, min(1.0, u))


def ease_in_out_sine(u: float) -> float:
    # Smooth easing with zero slope at ends
    import math
    u = max(0.0, min(1.0, u))
    return 0.5 - 0.5 * math.cos(math.pi * u)


def compute_leg_targets_for_direction(
    hip_low_high: Dict[int, Tuple[int, int]],
    direction: str,
) -> Tuple[Dict[int, int], Dict[int, int]]:
    """Return (stance_target, place_target) per hip based on direction."""
    stance_target: Dict[int, int] = {}
    place_target: Dict[int, int] = {}
    reversed_hips = {3, 7}  # invert swing direction for these hips
    for hip_id, (low, high) in hip_low_high.items():
        if hip_id in reversed_hips:
            # Swap mapping for reversed hips
            if direction == "forward":
                stance_target[hip_id] = high
                place_target[hip_id] = low
            else:  # backward
                stance_target[hip_id] = low
                place_target[hip_id] = high
        else:
            if direction == "forward":
                stance_target[hip_id] = low   # back
                place_target[hip_id] = high  # forward
            else:  # backward
                stance_target[hip_id] = high  # forward becomes stance
                place_target[hip_id] = low    # back becomes placement
    return stance_target, place_target


def main() -> None:
    parser = argparse.ArgumentParser(description="Spider gait: one leg at a time crawl (smooth control loop)")
    parser.add_argument("--port", required=True, help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Baudrate (default 1000000)")
    parser.add_argument("--direction", choices=["forward", "backward"], default="forward", help="Gait direction")
    parser.add_argument("--cycle-time", type=float, default=2.5, help="Total cycle duration in seconds")
    parser.add_argument("--swing-frac", type=float, default=0.2, help="Fraction of cycle for swing phase per leg (<=0.25)")
    parser.add_argument("--tick-hz", type=float, default=30.0, help="Command update rate (Hz)")
    parser.add_argument("--speed", type=int, default=2600, help="Commanded speed for moves")
    parser.add_argument("--acc", type=int, default=60, help="Commanded acceleration for moves")
    parser.add_argument("--lift-scale", type=float, default=1.0, help="Scale for knee lift (0..1) toward max")
    parser.add_argument("--ranges-file", default=os.path.join(THIS_DIR, "servo_ranges.json"), help="Path to ranges JSON file")
    args = parser.parse_args()

    # Load ranges
    ranges = parse_ranges_file(args.ranges_file)
    # Validate all 8 servos present
    ensure_all_present(ranges, [1, 2, 3, 4, 5, 6, 7, 8])
    hip_low_high = compute_hip_low_high(ranges)

    # Knee mid/max map
    knee_mid: Dict[int, int] = {sid: int(ranges[sid]["mid"]) for sid in (2, 4, 6, 8)}
    knee_max: Dict[int, int] = {sid: int(ranges[sid]["max"]) for sid in (2, 4, 6, 8)}

    # Build leg order and per-leg phase offsets (ensure only one leg swings at a time)
    legs = build_leg_order(args.direction)
    # Enforce swing fraction safety
    swing_frac = max(0.05, min(0.25, float(args.swing_frac)))
    # Phase offsets equally spaced across cycle for 4 legs
    leg_offsets: Dict[Leg, float] = {
        legs[0]: 0.0,
        legs[1]: 0.25,
        legs[2]: 0.50,
        legs[3]: 0.75,
    }

    manager = ServoManager()
    ok, err = manager.connect(args.port, args.baudrate)
    if not ok:
        print("Connect failed:", err)
        sys.exit(1)

    print(f"Connected. Starting {args.direction} crawl. Ctrl+C to stop.")
    try:
        # Torque on for all involved servos
        for sid in [1, 2, 3, 4, 5, 6, 7, 8]:
            try:
                manager.set_torque(sid, True)
            except Exception:
                pass

        # Targets for direction
        stance_target_for, place_target_for = compute_leg_targets_for_direction(hip_low_high, args.direction)

        # Initialize to stance positions with knees mid
        for hip_id in (1, 3, 5, 7):
            manager.write_position(hip_id, stance_target_for[hip_id], args.speed, args.acc)
        for knee_id in (2, 4, 6, 8):
            manager.write_position(knee_id, knee_mid[knee_id], args.speed, args.acc)
        time.sleep(0.3)

        # Time-based control loop
        dt = 1.0 / max(5.0, float(args.tick_hz))
        t = 0.0
        T = max(0.8, float(args.cycle_time))
        lift_scale = max(0.0, min(1.0, float(args.lift_scale)))

        import math
        def mod1(x: float) -> float:
            y = x - math.floor(x)
            return y

        while True:
            for hip_id, knee_id in legs:
                # Phase in [0,1) for this leg
                phase = mod1((t / T) + leg_offsets[(hip_id, knee_id)])
                if phase < swing_frac:
                    # Swing: hip moves from stance -> place, knee lifts toward max and back (sine-shaped)
                    u = phase / swing_frac
                    u_e = ease_in_out_sine(u)
                    hip_pos_f = lerp(float(stance_target_for[hip_id]), float(place_target_for[hip_id]), u_e)
                    # Knee lift: mid -> max at u=0.5 -> mid at u=1
                    s = math.sin(math.pi * u)
                    knee_pos_f = float(knee_mid[knee_id]) + s * (float(knee_max[knee_id]) - float(knee_mid[knee_id])) * lift_scale
                else:
                    # Stance: hip creeps back from place -> stance, knee stays at mid
                    v = (phase - swing_frac) / (1.0 - swing_frac)
                    hip_pos_f = lerp(float(place_target_for[hip_id]), float(stance_target_for[hip_id]), v)
                    knee_pos_f = float(knee_mid[knee_id])

                manager.write_position(hip_id, int(round(hip_pos_f)), args.speed, args.acc)
                manager.write_position(knee_id, int(round(knee_pos_f)), args.speed, args.acc)

            time.sleep(dt)
            t += dt

    except KeyboardInterrupt:
        print("\nStopping gait... Moving to neutral.")
        try:
            # Move all hips and knees to their mid positions (neutral)
            for hip_id in (1, 3, 5, 7):
                manager.write_position(hip_id, int(ranges[hip_id]["mid"]), args.speed, args.acc)
            for knee_id in (2, 4, 6, 8):
                manager.write_position(knee_id, int(ranges[knee_id]["mid"]), args.speed, args.acc)
            time.sleep(0.4)
        except Exception:
            pass
    finally:
        try:
            for sid in [1, 2, 3, 4, 5, 6, 7, 8]:
                manager.set_torque(sid, False)
        except Exception:
            pass
        manager.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()


