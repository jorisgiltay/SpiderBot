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


def build_leg_order(direction: str, order: str = "diagonal") -> List[Leg]:
    """
    Crawl order used for phase offsets.
    - crawl: FL → FR → RL → RR
    - diagonal: FL → RR → FR → RL
    """
    if order == "diagonal":
        legs: List[Leg] = [
            (1, 2),  # Front Left
            (7, 8),  # Rear Right
            (3, 4),  # Front Right
            (5, 6),  # Rear Left
        ]
    else:
        legs = [
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
    reversed_hips: List[int],
) -> Tuple[Dict[int, int], Dict[int, int]]:
    """Return (stance_target, place_target) per hip based on direction."""
    stance_target: Dict[int, int] = {}
    place_target: Dict[int, int] = {}
    reversed_set = set(reversed_hips)
    for hip_id, (low, high) in hip_low_high.items():
        if hip_id in reversed_set:
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


def scale_hip_targets(
    hip_low_high: Dict[int, Tuple[int, int]],
    ranges: Dict[int, Dict[str, int]],
    scale: float,
) -> Dict[int, Tuple[int, int]]:
    """Scale hip sweep around mid by factor (0..1). 1.0 = current, 0.5 = half."""
    s = max(0.1, min(1.0, float(scale)))
    out: Dict[int, Tuple[int, int]] = {}
    for hip_id, (low, high) in hip_low_high.items():
        mid = int(ranges[hip_id]["mid"])
        low_s = int(round(mid - s * (mid - int(low))))
        high_s = int(round(mid + s * (int(high) - mid)))
        out[hip_id] = (low_s, high_s)
    return out


def degrees_to_ticks(deg: float) -> int:
    # Approx 0..4095 maps to 0..360 degrees
    return int(round(deg * (4095.0 / 360.0)))


def enforce_hip_deg_limit(
    hip_low_high: Dict[int, Tuple[int, int]],
    ranges: Dict[int, Dict[str, int]],
    max_deg: float,
) -> Dict[int, Tuple[int, int]]:
    out: Dict[int, Tuple[int, int]] = {}
    delta = degrees_to_ticks(max_deg)
    for hip_id, (low, high) in hip_low_high.items():
        mid = int(ranges[hip_id]["mid"])
        low_l = max(int(ranges[hip_id]["min"]), mid - delta)
        high_l = min(int(ranges[hip_id]["max"]), mid + delta)
        # Also clamp within existing low/high just in case
        low_l = max(low_l, min(low, high))
        high_l = min(high_l, max(low, high))
        out[hip_id] = (low_l, high_l)
    return out


def ticks_to_rad_ticks_mapping(mid: int) -> Tuple[float, int]:
    # Returns (ticks_per_rad, mid)
    ticks_per_rad = 4095.0 / (2.0 * 3.141592653589793)
    return ticks_per_rad, mid


def knee_ik_ticks(
    hip_ticks: int,
    knee_mid_ticks: int,
    knee_sign: int,
    hip_mid_ticks: int,
    hip_link_mm: float,
    knee_link_mm: float,
    hip_height_mm: float,
    stance_knee_bias_ticks: float,
) -> int:
    """
    Simple planar IK in sagittal plane:
    - hip angle from hip_ticks relative to hip_mid
    - desired foot: y=0 (ground), x = hip_link*cos(hip) + knee_link*cos(knee_rel)
    We solve knee approx by keeping knee such that ankle reaches ground with given hip height.
    Here we use a simplified law: knee angle set so that vertical projection equals hip_height.
    """
    import math
    ticks_per_rad, _ = ticks_to_rad_ticks_mapping(hip_mid_ticks)
    hip_rad = (hip_ticks - hip_mid_ticks) / ticks_per_rad
    # Desired knee relative angle via 2-link geometry keeping end effector near ground at hip height
    # y = hip_link*sin(hip) + knee_link*sin(knee)
    # target y ≈ -hip_height (down). Solve knee via arcsin clamp.
    target_y = -float(hip_height_mm)
    y_hip = hip_link_mm * math.sin(hip_rad)
    y_needed_from_knee = target_y - y_hip
    # Clamp to reachable range [-knee_link, knee_link]
    s = max(-1.0, min(1.0, y_needed_from_knee / max(1e-6, knee_link_mm)))
    knee_rad = math.asin(s)
    knee_ticks = knee_mid_ticks + int(round(knee_sign * knee_rad * ticks_per_rad)) + int(round(stance_knee_bias_ticks))
    return knee_ticks


def move_all_to_neutral(
    manager: ServoManager,
    ranges: Dict[int, Dict[str, int]],
    speed: int,
    acc: int,
    tolerance: int = 8,
    timeout_s: float = 2.0,
) -> None:
    """Move all hips/knees to mid and wait until they settle or timeout."""
    target_by_id: Dict[int, int] = {sid: int(ranges[sid]["mid"]) for sid in (1, 2, 3, 4, 5, 6, 7, 8)}
    # Send commands
    for sid in (1, 2, 3, 4, 5, 6, 7, 8):
        try:
            manager.set_torque(sid, True)
            manager.write_position(sid, target_by_id[sid], speed, acc)
        except Exception:
            pass
    # Wait until all within tolerance or timeout
    start = time.time()
    remaining = set(target_by_id.keys())
    while remaining and (time.time() - start) < timeout_s:
        done_now = []
        for sid in list(remaining):
            try:
                st, err = manager.read_status(sid)
                if err is None and st:
                    pres = int(st.get("present_position", 1 << 30))
                    if abs(pres - target_by_id[sid]) <= tolerance:
                        done_now.append(sid)
            except Exception:
                # If we can't read, assume it's done to avoid hanging
                done_now.append(sid)
        for sid in done_now:
            if sid in remaining:
                remaining.remove(sid)
        time.sleep(0.05)
    # Small settle pause
    time.sleep(0.15)


def main() -> None:
    parser = argparse.ArgumentParser(description="Spider gait: one leg at a time crawl (smooth control loop)")
    parser.add_argument("--port", required=True, help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Baudrate (default 1000000)")
    parser.add_argument("--direction", choices=["forward", "backward"], default="forward", help="Gait direction")
    parser.add_argument("--cycle-time", type=float, default=2.8, help="Total cycle duration in seconds")
    parser.add_argument("--swing-frac", type=float, default=0.2, help="Fraction of cycle for swing phase per leg (<=0.25)")
    parser.add_argument("--tick-hz", type=float, default=30.0, help="Command update rate (Hz)")
    parser.add_argument("--speed", type=int, default=2600, help="Commanded speed for moves")
    parser.add_argument("--acc", type=int, default=60, help="Commanded acceleration for moves")
    parser.add_argument("--lift-scale", type=float, default=1.0, help="Scale for knee lift (0..1) toward max")
    parser.add_argument("--hip-swing-scale", type=float, default=0.7, help="Scale hip sweep around mid (0..1). 1.0 ≈ 90° total sweep")
    parser.add_argument("--hip-max-deg", type=float, default=60.0, help="Clamp hip sweep to ±this many degrees around mid")
    parser.add_argument("--stance-knee-bias", type=float, default=0.15, help="Lower stance knees by this fraction of (mid-min), 0..0.5")
    parser.add_argument("--stance-hold-frac", type=float, default=0.25, help="Hold hips wide for this fraction at start of stance, 0..0.6")
    parser.add_argument("--knee-hip-gain", type=float, default=0.6, help="Stance knee coupling gain with hip motion (ticks per tick)")
    parser.add_argument("--knee-band-frac", type=float, default=0.2, help="Limit knee motion to this fraction of total range (0..1)")
    parser.add_argument("--knee-band-anchor", choices=["min", "max"], default="max", help="Anchor the limited knee range at min (low) or max (high)")
    # IK options for stance
    parser.add_argument("--ik-stance", action="store_true", help="Use 2-link IK in stance to keep foot on ground")
    parser.add_argument("--hip-link-mm", type=float, default=50.0, help="Hip→knee link length (mm)")
    parser.add_argument("--knee-link-mm", type=float, default=81.0, help="Knee→foot effective length (mm)")
    parser.add_argument("--hip-height-mm", type=float, default=60.0, help="Desired hip height above ground during stance (mm)")
    parser.add_argument("--knee-sign", type=int, choices=[-1,1], default=1, help="+1 or -1 mapping from knee angle to ticks")
    parser.add_argument("--knee-sign-override", type=str, default="", help="Per-knee overrides like 2:-1,4:-1 (comma-separated)")
    parser.add_argument("--reverse-hips", type=str, default="3,7", help="Comma-separated hip IDs with reversed swing direction")
    # Knee swing direction per knee: 'up' means toward max (default) or toward min
    parser.add_argument("--knee-swing-up", type=str, default="2:+,4:+,6:+,8:+", help="Per-knee swing direction (+ toward max, - toward min), e.g., 8:-")
    parser.add_argument("--order", choices=["crawl", "diagonal"], default="diagonal", help="Leg stepping order")
    parser.add_argument("--ranges-file", default=os.path.join(THIS_DIR, "servo_ranges.json"), help="Path to ranges JSON file")
    parser.add_argument("--gait-mode", choices=["crawl", "trot"], default="crawl", help="Crawl = one leg swing; Trot = diagonal pairs swing together")
    parser.add_argument("--crawl-style", choices=["smooth", "discrete"], default="smooth", help="For crawl: smooth = others creep in stance; discrete = others hold still")
    args = parser.parse_args()

    # Load ranges
    ranges = parse_ranges_file(args.ranges_file)
    # Validate all 8 servos present
    ensure_all_present(ranges, [1, 2, 3, 4, 5, 6, 7, 8])
    hip_low_high = compute_hip_low_high(ranges)
    # Optionally scale hip sweep amplitude then enforce degree cap
    hip_low_high = scale_hip_targets(hip_low_high, ranges, args.hip_swing_scale)
    hip_low_high = enforce_hip_deg_limit(hip_low_high, ranges, args.hip_max_deg)

    # Knee min/mid/max maps
    knee_min: Dict[int, int] = {sid: int(ranges[sid]["min"]) for sid in (2, 4, 6, 8)}
    knee_mid: Dict[int, int] = {sid: int(ranges[sid]["mid"]) for sid in (2, 4, 6, 8)}
    knee_max: Dict[int, int] = {sid: int(ranges[sid]["max"]) for sid in (2, 4, 6, 8)}

    # Knee band helpers
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
        if kmid < bmin:
            knee_mid[sid] = bmin
        if kmid > bmax:
            knee_mid[sid] = bmax
        return (min(bmin, bmax), max(bmin, bmax))

    def clamp_knee(sid: int, val: float) -> int:
        bmin, bmax = knee_band(sid)
        return int(min(max(int(round(val)), bmin), bmax))

    # Build leg order and per-leg phase offsets
    legs = build_leg_order(args.direction, args.order)
    # Enforce swing fraction safety
    swing_frac = max(0.05, min(0.40 if args.gait_mode == "trot" else 0.25, float(args.swing_frac)))
    if args.gait_mode == "trot":
        # Diagonal pairs swing together: (FL,RR) then (FR,RL)
        # Use two phases 0.0 and 0.5
        leg_offsets: Dict[Leg, float] = {
            (1, 2): 0.0,
            (7, 8): 0.0,
            (3, 4): 0.5,
            (5, 6): 0.5,
        }
    else:
        # Crawl: four phases 0.0, 0.25, 0.5, 0.75
        leg_offsets = {
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
        # Parse reversed hips from CLI
        rev_hips: List[int] = []
        if args.reverse_hips.strip():
            for tok in args.reverse_hips.split(","):
                tok = tok.strip()
                if tok.isdigit():
                    rev_hips.append(int(tok))
        if not rev_hips:
            rev_hips = [3, 7]
        stance_target_for, place_target_for = compute_leg_targets_for_direction(hip_low_high, args.direction, rev_hips)

        # Initialize to stance positions with knees mid (clamped to band)
        for hip_id in (1, 3, 5, 7):
            manager.write_position(hip_id, stance_target_for[hip_id], args.speed, args.acc)
        for knee_id in (2, 4, 6, 8):
            manager.write_position(knee_id, clamp_knee(knee_id, float(knee_mid[knee_id])), args.speed, args.acc)
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
            if args.gait_mode == "crawl" and args.crawl_style == "discrete":
                # Discrete crawl: move one leg through full swing while others hold their current stance positions
                # Parse swing directions
                swing_dir: Dict[int, int] = {2: 1, 4: 1, 6: 1, 8: 1}
                try:
                    if args.knee_swing_up:
                        for tok in args.knee_swing_up.split(","):
                            t = tok.strip()
                            if not t:
                                continue
                            if ":" in t:
                                k_str, s_str = t.split(":", 1)
                                if k_str.strip().isdigit():
                                    k = int(k_str.strip())
                                    if s_str.strip().startswith("-"):
                                        swing_dir[k] = -1
                                    else:
                                        swing_dir[k] = 1
                except Exception:
                    pass

                for hip_id, knee_id in legs:
                    # Ensure others are at their stance target and knees at biased mid
                    for other_hip, other_knee in legs:
                        if other_hip == hip_id:
                            continue
                        # Hold hips at current stance target
                        manager.write_position(other_hip, int(stance_target_for[other_hip]), args.speed, args.acc)
                        # Hold knees slightly biased down within band
                        bias = max(0.0, min(0.5, float(args.stance_knee_bias)))
                        knee_hold = float(knee_mid[other_knee]) - bias * (float(knee_mid[other_knee]) - float(knee_min[other_knee]))
                        manager.write_position(other_knee, clamp_knee(other_knee, knee_hold), args.speed, args.acc)

                    # Move the active leg through swing
                    swing_steps = max(10, int(round(swing_frac * T * args.tick_hz)))
                    for i in range(swing_steps):
                        u = (i + 1) / swing_steps
                        u_e = ease_in_out_sine(u)
                        hip_pos_f = lerp(float(stance_target_for[hip_id]), float(place_target_for[hip_id]), u_e)
                        s = math.sin(math.pi * u)
                        # Choose knee lift direction per knee
                        if swing_dir.get(knee_id, 1) >= 0:
                            knee_span = float(knee_max[knee_id]) - float(knee_mid[knee_id])
                            knee_pos_f = float(knee_mid[knee_id]) + s * knee_span * lift_scale
                        else:
                            knee_span = float(knee_mid[knee_id]) - float(knee_min[knee_id])
                            knee_pos_f = float(knee_mid[knee_id]) - s * knee_span * lift_scale
                        knee_cmd = clamp_knee(knee_id, knee_pos_f)
                        manager.write_position(hip_id, int(round(hip_pos_f)), args.speed, args.acc)
                        manager.write_position(knee_id, knee_cmd, args.speed, args.acc)
                        time.sleep(dt)

                    # Brief settle in place before moving to next leg
                    time.sleep(dt * 2)
            else:
                # Smooth loop: original continuous creeping stance
                # Parse swing directions
                swing_dir: Dict[int, int] = {2: 1, 4: 1, 6: 1, 8: 1}
                try:
                    if args.knee_swing_up:
                        for tok in args.knee_swing_up.split(","):
                            t = tok.strip()
                            if not t:
                                continue
                            if ":" in t:
                                k_str, s_str = t.split(":", 1)
                                if k_str.strip().isdigit():
                                    k = int(k_str.strip())
                                    if s_str.strip().startswith("-"):
                                        swing_dir[k] = -1
                                    else:
                                        swing_dir[k] = 1
                except Exception:
                    pass

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
                        if swing_dir.get(knee_id, 1) >= 0:
                            knee_span = float(knee_max[knee_id]) - float(knee_mid[knee_id])
                            knee_pos_f = float(knee_mid[knee_id]) + s * knee_span * lift_scale
                        else:
                            knee_span = float(knee_mid[knee_id]) - float(knee_min[knee_id])
                            knee_pos_f = float(knee_mid[knee_id]) - s * knee_span * lift_scale
                    else:
                        # Stance: hip creeps back from place -> stance; use IK (optional) to keep foot on ground
                        v = (phase - swing_frac) / (1.0 - swing_frac)
                        # Hold wide base for early stance to increase stability
                        hold = max(0.0, min(0.6, float(args.stance_hold_frac)))
                        if v < hold:
                            hip_pos_f = float(place_target_for[hip_id])
                        else:
                            v2 = (v - hold) / (1.0 - hold)
                            hip_pos_f = lerp(float(place_target_for[hip_id]), float(stance_target_for[hip_id]), v2)
                        # Base stance knee bias ticks
                        bias = max(0.0, min(0.5, float(args.stance_knee_bias)))
                        stance_knee_f = float(knee_mid[knee_id]) - bias * (float(knee_mid[knee_id]) - float(knee_min[knee_id]))
                        if args.ik_stance:
                            # Per-knee sign override
                            knee_sign = int(args.knee_sign)
                            if args.knee_sign_override:
                                for pair in args.knee_sign_override.split(","):
                                    pair = pair.strip()
                                    if not pair:
                                        continue
                                    if ":" in pair:
                                        k_str, s_str = pair.split(":", 1)
                                        if k_str.strip().isdigit():
                                            if int(k_str.strip()) == int(knee_id):
                                                try:
                                                    knee_sign = int(s_str.strip())
                                                except Exception:
                                                    pass
                            knee_cmd = knee_ik_ticks(
                                hip_ticks=int(round(hip_pos_f)),
                                knee_mid_ticks=int(knee_mid[knee_id]),
                                knee_sign=knee_sign,
                                hip_mid_ticks=int(ranges[hip_id]["mid"]),
                                hip_link_mm=float(args.hip_link_mm),
                                knee_link_mm=float(args.knee_link_mm),
                                hip_height_mm=float(args.hip_height_mm),
                                stance_knee_bias_ticks=stance_knee_f - float(knee_mid[knee_id]),
                            )
                            knee_cmd = clamp_knee(knee_id, knee_cmd)
                        else:
                            # Linear coupling fallback
                            hip_stance = float(stance_target_for[hip_id])
                            hip_disp = float(hip_pos_f) - hip_stance
                            knee_pos_f = stance_knee_f + float(args.knee_hip_gain) * hip_disp
                            knee_cmd = clamp_knee(knee_id, knee_pos_f)

                    manager.write_position(hip_id, int(round(hip_pos_f)), args.speed, args.acc)
                    manager.write_position(knee_id, knee_cmd if 'knee_cmd' in locals() else int(round(knee_pos_f)), args.speed, args.acc)

                time.sleep(dt)
                t += dt

    except KeyboardInterrupt:
        print("\nStopping gait... Moving to neutral.")
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


