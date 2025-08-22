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
    # Map height in [0, ~60] mm to knee ticks where:
    # 0 mm  -> max_ticks (fully bent, lying low)
    # 60 mm -> base_mid  (max practical height per JSON "mid")
    max_height_mm = 60.0
    h = max(0.0, min(max_height_mm, float(height_mm)))
    t = h / max_height_mm  # 0..1
    # Linear interpolation between max_ticks (low) and base_mid (high)
    target = int(round((1.0 - t) * max_ticks + t * base_mid))
    # Safety clamp to servo's physical [min,max]
    return clamp_ticks(target, min_ticks, max_ticks)


def main() -> int:
    parser = argparse.ArgumentParser(description="Adjust body height by moving all knee servos.")
    parser.add_argument("--port", required=False, help="Serial port (e.g. COM3). If omitted, first detected is used.")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Baudrate (default: 1_000_000)")
    parser.add_argument("--speed", type=int, default=2400, help="Servo speed (ticks/s)")
    parser.add_argument("--acc", type=int, default=50, help="Servo acceleration")
    parser.add_argument("--height", type=float, default=0.0, help="Body height in mm (0..~60, 0=ground, ~60=max)")
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


