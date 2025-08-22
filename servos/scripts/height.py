#!/usr/bin/env python
import argparse
import json
import os
import sys
import time


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


def map_height_to_knee_ticks(height_mm: float, base_mid: int, min_ticks: int, max_ticks: int) -> int:
    # Simple linear mapping example: define a plausible range of height adjustments
    # height_mm in [-40, +40] maps to knee ticks offset in [knee_extend, knee_bend]
    # Positive height reduces knee bend (extends), negative height increases bend.
    height_mm = max(-40.0, min(40.0, height_mm))
    # Compute offset: -40 -> +delta, +40 -> -delta
    delta = (max_ticks - min_ticks) * 0.15  # use 15% of knee range as adjustable window
    # Normalize height to [-1, 1]
    n = height_mm / 40.0
    # Offset ticks (invert because +height means extending -> lower ticks if min corresponds to extension)
    offset = int(round(-n * delta))
    return clamp_ticks(base_mid + offset, min_ticks, max_ticks)


def main() -> int:
    parser = argparse.ArgumentParser(description="Adjust body height by moving all knee servos.")
    parser.add_argument("--port", required=False, help="Serial port (e.g. COM3). If omitted, first detected is used.")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Baudrate (default: 1_000_000)")
    parser.add_argument("--speed", type=int, default=2400, help="Servo speed (ticks/s)")
    parser.add_argument("--acc", type=int, default=50, help="Servo acceleration")
    parser.add_argument("--height", type=float, default=0.0, help="Relative height in mm (approx, -40..+40)")
    parser.add_argument("--hold", type=float, default=0.8, help="Seconds to hold the pose before exit")
    args = parser.parse_args()

    ranges = load_servo_ranges()["servos"]

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
        # Keep hips neutral while changing height
        hip_ids = [1, 3, 5, 7]
        knee_ids = [2, 4, 6, 8]

        for sid in hip_ids:
            s = ranges[f"servo{sid}"]
            pos = clamp_ticks(int(s["mid"]), int(s["min"]), int(s["max"]))
            ok, err = manager.write_position(sid, pos, args.speed, args.acc)
            if not ok:
                print(f"Hip {sid} write failed: {err}")

        for sid in knee_ids:
            s = ranges[f"servo{sid}"]
            target = map_height_to_knee_ticks(float(args.height), int(s["mid"]), int(s["min"]), int(s["max"]))
            ok, err = manager.write_position(sid, target, args.speed, args.acc)
            if not ok:
                print(f"Knee {sid} write failed: {err}")

        time.sleep(max(0.0, float(args.hold)))
        return 0
    finally:
        manager.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())


