#!/usr/bin/env python
import argparse
import json
import os
import sys
import time
from typing import Dict, Tuple


# Make webInterface importable
THIS_DIR = os.path.dirname(__file__)
WEB_IF_DIR = os.path.join(THIS_DIR, "webInterface")
if WEB_IF_DIR not in sys.path:
    sys.path.append(WEB_IF_DIR)

from servo_manager import ServoManager  # type: ignore  # noqa: E402


def parse_ranges_file(ranges_path: str) -> Dict[int, Dict[str, int]]:
    """
    Parse servo ranges from a JSON file. Expected formats:
    1) {"servos": {"servo1": {"min":int, "mid":int, "max":int}, ...}}
    2) {"1": {"min":int, ...}, "2": {...}} or {1: {...}}
    Falls back to the legacy plain-text parser if JSON load fails.
    Returns {1: {"min":..,"mid":..,"max":..}, 2: {...}}
    """
    ranges: Dict[int, Dict[str, int]] = {}
    if not os.path.exists(ranges_path):
        return ranges
    try:
        with open(ranges_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        # Support either nested under "servos" or at top-level
        maybe_servos = data.get("servos", data) if isinstance(data, dict) else {}
        if isinstance(maybe_servos, dict):
            for key, cfg in maybe_servos.items():
                # Convert key like "servo1" or "1" to int id
                sid: int = -1
                if isinstance(key, int):
                    sid = key
                elif isinstance(key, str):
                    digits = "".join(ch for ch in key if ch.isdigit())
                    if digits:
                        try:
                            sid = int(digits)
                        except Exception:
                            sid = -1
                if sid == -1 or not isinstance(cfg, dict):
                    continue
                try:
                    mn = int(cfg.get("min"))
                    md = int(cfg.get("mid"))
                    mx = int(cfg.get("max"))
                except Exception:
                    continue
                ranges[sid] = {"min": mn, "mid": md, "max": mx}
        return ranges
    except Exception:
        # Legacy fallback: parse the old plain-text format
        legacy: Dict[int, Dict[str, int]] = {}
        current: int = -1
        try:
            with open(ranges_path, "r", encoding="utf-8") as f:
                for raw in f:
                    line = raw.strip().lower()
                    if not line:
                        continue
                    if line.startswith("servo"):
                        # Extract first integer on the line
                        digits = "".join(ch for ch in line if ch.isdigit())
                        current = int(digits) if digits else -1
                        if current != -1 and current not in legacy:
                            legacy[current] = {}
                        continue
                    if current == -1:
                        continue
                    for key in ("min", "mid", "max"):
                        if line.startswith(f"{key}:"):
                            try:
                                value = int(line.split(":", 1)[1].strip())
                                legacy[current][key] = value
                            except Exception:
                                pass
        except Exception:
            return {}
        return legacy


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def compute_hip_targets(hip_min: int, hip_mid: int, hip_max: int) -> Tuple[int, int]:
    """
    Hip should only use 90° out of its 180° range, centered at mid.
    That is ±(range/4) around mid in raw position units.
    """
    quarter_range = int(round((hip_max - hip_min) / 4.0))
    low = clamp(hip_mid - quarter_range, hip_min, hip_max)
    high = clamp(hip_mid + quarter_range, hip_min, hip_max)
    return low, high


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a simple one-leg gait loop (hip+knee)")
    parser.add_argument("--port", required=True, help="Serial port (e.g., COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Baudrate (default 1000000)")
    parser.add_argument("--hip-id", type=int, default=1, help="Servo ID for hip (default 1)")
    parser.add_argument("--knee-id", type=int, default=2, help="Servo ID for knee (default 2)")
    parser.add_argument("--step-delay", type=float, default=0.6, help="Seconds to wait after each phase")
    parser.add_argument("--speed", type=int, default=2400, help="Commanded speed for moves")
    parser.add_argument("--acc", type=int, default=50, help="Commanded acceleration for moves")
    parser.add_argument("--ranges-file", default=os.path.join(THIS_DIR, "servo_ranges.json"), help="Path to ranges JSON file")
    args = parser.parse_args()

    ranges = parse_ranges_file(args.ranges_file)
    if 1 not in ranges or 2 not in ranges:
        print("Error: Could not parse both servo 1 and servo 2 ranges from:", args.ranges_file)
        sys.exit(1)

    hip_min = ranges[1].get("min")
    hip_mid = ranges[1].get("mid")
    hip_max = ranges[1].get("max")
    knee_min = ranges[2].get("min")  # not used per spec
    knee_mid = ranges[2].get("mid")
    knee_max = ranges[2].get("max")
    for v_name, v in (
        ("hip_min", hip_min), ("hip_mid", hip_mid), ("hip_max", hip_max),
        ("knee_mid", knee_mid), ("knee_max", knee_max),
    ):
        if v is None:
            print(f"Error: Missing {v_name} in ranges file")
            sys.exit(1)

    hip_low, hip_high = compute_hip_targets(int(hip_min), int(hip_mid), int(hip_max))

    manager = ServoManager()
    ok, err = manager.connect(args.port, args.baudrate)
    if not ok:
        print("Connect failed:", err)
        sys.exit(1)

    print("Connected. Starting gait loop. Ctrl+C to stop.")
    try:
        # Ensure torque is on before starting
        manager.set_torque(args.hip_id, True)
        manager.set_torque(args.knee_id, True)

        while True:
            # Phase 1: Hip back, knee at mid (stance start)
            manager.write_position(args.hip_id, hip_low, args.speed, args.acc)
            manager.write_position(args.knee_id, int(knee_mid), args.speed, args.acc)
            time.sleep(args.step_delay)

            # Phase 2: Lift knee (to max), hip stays back
            manager.write_position(args.knee_id, int(knee_max), args.speed, args.acc)
            time.sleep(args.step_delay)

            # Phase 3: Swing hip forward, knee stays lifted
            manager.write_position(args.hip_id, hip_high, args.speed, args.acc)
            time.sleep(args.step_delay)

            # Phase 4: Place knee back to mid, hip forward
            manager.write_position(args.knee_id, int(knee_mid), args.speed, args.acc)
            time.sleep(args.step_delay)

    except KeyboardInterrupt:
        print("\nStopping gait loop...")
    finally:
        try:
            manager.set_torque(args.hip_id, False)
            manager.set_torque(args.knee_id, False)
        except Exception:
            pass
        manager.disconnect()
        print("Disconnected.")


if __name__ == "__main__":
    main()


