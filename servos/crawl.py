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


def build_leg_order(order: str = "diagonal") -> List[Leg]:
    if order == "diagonal":
        return [(1, 2), (7, 8), (3, 4), (5, 6)]
    return [(1, 2), (3, 4), (5, 6), (7, 8)]


def compute_hip_low_high(ranges: Dict[int, Dict[str, int]]) -> Dict[int, Tuple[int, int]]:
    result: Dict[int, Tuple[int, int]] = {}
    for hip_id in (1, 3, 5, 7):
        r = ranges.get(hip_id, {})
        if not {"min", "mid", "max"}.issubset(r.keys()):
            raise ValueError(f"Missing min/mid/max for hip servo {hip_id}")
        low, high = compute_hip_targets(int(r["min"]), int(r["mid"]), int(r["max"]))
        result[hip_id] = (low, high)
    return result


def scale_hip_targets(hip_low_high: Dict[int, Tuple[int, int]], ranges: Dict[int, Dict[str, int]], scale: float) -> Dict[int, Tuple[int, int]]:
    s = max(0.1, min(1.0, float(scale)))
    out: Dict[int, Tuple[int, int]] = {}
    for hip_id, (low, high) in hip_low_high.items():
        mid = int(ranges[hip_id]["mid"])
        low_s = int(round(mid - s * (mid - int(low))))
        high_s = int(round(mid + s * (int(high) - mid)))
        out[hip_id] = (low_s, high_s)
    return out


def move_all_to_neutral(manager: ServoManager, ranges: Dict[int, Dict[str, int]], speed: int, acc: int) -> None:
    target_by_id: Dict[int, int] = {sid: int(ranges[sid]["mid"]) for sid in (1, 2, 3, 4, 5, 6, 7, 8)}
    for sid in (1, 2, 3, 4, 5, 6, 7, 8):
        try:
            manager.set_torque(sid, True)
            manager.write_position(sid, target_by_id[sid], speed, acc)
        except Exception:
            pass
    time.sleep(0.4)


def main() -> None:
    parser = argparse.ArgumentParser(description="Discrete crawl gait with active CoG shift")
    parser.add_argument("--port", required=True, help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Baudrate (default 1000000)")
    parser.add_argument("--direction", choices=["forward", "backward"], default="forward", help="Gait direction")
    parser.add_argument("--order", choices=["crawl", "diagonal"], default="diagonal", help="Leg stepping order")
    parser.add_argument("--speed", type=int, default=2400, help="Commanded speed for moves")
    parser.add_argument("--acc", type=int, default=50, help="Commanded acceleration for moves")
    parser.add_argument("--hip-swing-scale", type=float, default=0.6, help="Scale hip sweep around mid (0..1)")
    parser.add_argument("--lift-scale", type=float, default=0.8, help="Scale knee lift height (0..1)")
    parser.add_argument("--knee-band-frac", type=float, default=0.2, help="Limit knee motion to this fraction of total range (0..1)")
    parser.add_argument("--knee-band-anchor", choices=["min", "max"], default="min", help="Anchor the limited knee range at min (low) or max (high)")
    parser.add_argument("--stance-knee-bias", type=float, default=0.25, help="Lower stance knees by this fraction of (mid-min), 0..0.5")
    parser.add_argument("--cog-drop", type=float, default=0.15, help="Extra drop on diagonal support knee during swing, fraction of (mid-min)")
    parser.add_argument("--swing-time", type=float, default=0.7, help="Seconds to perform a leg swing")
    parser.add_argument("--settle-time", type=float, default=0.15, help="Seconds to settle after leg placement")
    parser.add_argument("--ranges-file", default=os.path.join(THIS_DIR, "servo_ranges.json"), help="Path to ranges JSON file")
    args = parser.parse_args()

    ranges = parse_ranges_file(args.ranges_file)
    knee_min: Dict[int, int] = {sid: int(ranges[sid]["min"]) for sid in (2, 4, 6, 8)}
    knee_mid: Dict[int, int] = {sid: int(ranges[sid]["mid"]) for sid in (2, 4, 6, 8)}
    knee_max: Dict[int, int] = {sid: int(ranges[sid]["max"]) for sid in (2, 4, 6, 8)}

    # Define allowed knee band per joint to keep bot close to ground
    band_frac = max(0.05, min(1.0, float(args.knee_band_frac)))
    def knee_band(sid: int) -> Tuple[int, int]:
        kmin = knee_min[sid]; kmid = knee_mid[sid]; kmax = knee_max[sid]
        span = float(kmax - kmin)
        if args.knee_band_anchor == "min":
            bmin = kmin
            bmax = int(round(kmin + band_frac * span))
        else:
            bmin = int(round(kmax - band_frac * span))
            bmax = kmax
        # Ensure mid is inside band; if not, clamp toward band
        if kmid < bmin:
            kmid = bmin
        if kmid > bmax:
            kmid = bmax
        return (min(bmin, bmax), max(bmin, bmax))

    def clamp_knee(sid: int, val: float) -> int:
        bmin, bmax = knee_band(sid)
        return int(min(max(int(round(val)), bmin), bmax))

    hip_low_high = compute_hip_low_high(ranges)
    hip_low_high = scale_hip_targets(hip_low_high, ranges, args.hip_swing_scale)

    # Direction mapping (with hip 3 and 7 reversed)
    stance_target: Dict[int, int] = {}
    place_target: Dict[int, int] = {}
    for hip_id, (low, high) in hip_low_high.items():
        is_reversed = hip_id in {3, 7}
        if args.direction == "forward":
            stance_target[hip_id] = high if is_reversed else low
            place_target[hip_id] = low if is_reversed else high
        else:
            stance_target[hip_id] = low if is_reversed else high
            place_target[hip_id] = high if is_reversed else low

    legs = build_leg_order(args.order)

    manager = ServoManager()
    ok, err = manager.connect(args.port, args.baudrate)
    if not ok:
        print("Connect failed:", err)
        sys.exit(1)

    print("Connected. Starting discrete crawl with CoG shift. Ctrl+C to stop.")
    try:
        # Torque on
        for sid in [1, 2, 3, 4, 5, 6, 7, 8]:
            try:
                manager.set_torque(sid, True)
            except Exception:
                pass

        # Initialize: hips to stance, knees to biased stance
        bias = max(0.0, min(0.5, float(args.stance_knee_bias)))
        for hip_id in (1, 3, 5, 7):
            manager.write_position(hip_id, stance_target[hip_id], args.speed, args.acc)
        for knee_id in (2, 4, 6, 8):
            knee_hold = float(knee_mid[knee_id]) - bias * (float(knee_mid[knee_id]) - float(knee_min[knee_id]))
            manager.write_position(knee_id, clamp_knee(knee_id, knee_hold), args.speed, args.acc)
        time.sleep(0.3)

        # Per-leg discrete swing with CoG shift via diagonal knee drop
        dt = 1.0 / 30.0
        steps = max(10, int(round(args.swing_time / dt)))
        lift_scale = max(0.0, min(1.0, float(args.lift_scale)))
        cog_drop = max(0.0, min(0.4, float(args.cog_drop)))

        diag_map = {1: 8, 3: 6, 5: 4, 7: 2}  # diagonal knee id for a given hip id

        import math
        def ease(u: float) -> float:
            u = max(0.0, min(1.0, u))
            return 0.5 - 0.5 * math.cos(math.pi * u)

        while True:
            for hip_id, knee_id in legs:
                # CoG pre-shift: drop diagonal support knee a bit more
                diag_knee = diag_map[hip_id]
                knee_hold_diag = float(knee_mid[diag_knee]) - (bias + cog_drop) * (float(knee_mid[diag_knee]) - float(knee_min[diag_knee]))
                manager.write_position(diag_knee, clamp_knee(diag_knee, knee_hold_diag), args.speed, args.acc)

                # Swing the active leg
                for i in range(steps):
                    u = (i + 1) / steps
                    u_e = ease(u)
                    hip_pos_f = (1.0 - u_e) * float(stance_target[hip_id]) + u_e * float(place_target[hip_id])
                    s = math.sin(math.pi * u)
                    knee_pos_f = float(knee_mid[knee_id]) + s * (float(knee_max[knee_id]) - float(knee_mid[knee_id])) * lift_scale
                    knee_pos_i = clamp_knee(knee_id, knee_pos_f)
                    manager.write_position(hip_id, int(round(hip_pos_f)), args.speed, args.acc)
                    manager.write_position(knee_id, knee_pos_i, args.speed, args.acc)
                    time.sleep(dt)

                # Place and settle
                time.sleep(max(0.05, float(args.settle_time)))

                # Restore diagonal knee to normal stance bias
                knee_hold_diag2 = float(knee_mid[diag_knee]) - bias * (float(knee_mid[diag_knee]) - float(knee_min[diag_knee]))
                manager.write_position(diag_knee, clamp_knee(diag_knee, knee_hold_diag2), args.speed, args.acc)

    except KeyboardInterrupt:
        print("\nStopping crawl... Moving to neutral.")
        try:
            move_all_to_neutral(manager, ranges, args.speed, args.acc)
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


