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


def main() -> int:
    parser = argparse.ArgumentParser(description="Set a safe standing pose for the quadruped.")
    parser.add_argument("--port", required=False, help="Serial port (e.g. COM3). If omitted, first detected is used.")
    parser.add_argument("--baud", type=int, default=1_000_000, help="Baudrate (default: 1_000_000)")
    parser.add_argument("--speed", type=int, default=2400, help="Servo speed (ticks/s)")
    parser.add_argument("--acc", type=int, default=50, help="Servo acceleration")
    parser.add_argument(
        "--knee-offset",
        type=int,
        default=0,
        help="Additional ticks to add to knee 'mid' for fine-tuning stand height (+ bends, - extends, depends on setup)",
    )
    parser.add_argument("--hold", type=float, default=0.8, help="Seconds to hold the pose before exit")
    args = parser.parse_args()

    # Mapping assumption based on user's description
    # 1,2 = FL hip,knee | 3,4 = FR hip,knee | 5,6 = RL hip,knee | 7,8 = RR hip,knee
    hip_ids = [1, 3, 5, 7]
    knee_ids = [2, 4, 6, 8]

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

    try:
        # Set hips to mid
        for sid in hip_ids:
            s = ranges[f"servo{sid}"]
            pos = int(s["mid"])  # neutral hip
            pos = clamp_ticks(pos, int(s["min"]), int(s["max"]))
            ok, err = mgr.write_position(sid, pos, args.speed, args.acc)
            if not ok:
                print(f"Hip {sid} write failed: {err}")

        # Set knees to mid + offset (user-tunable)
        for sid in knee_ids:
            s = ranges[f"servo{sid}"]
            pos = int(s["mid"]) + int(args.knee_offset)
            pos = clamp_ticks(pos, int(s["min"]), int(s["max"]))
            ok, err = mgr.write_position(sid, pos, args.speed, args.acc)
            if not ok:
                print(f"Knee {sid} write failed: {err}")

        time.sleep(max(0.0, float(args.hold)))
        return 0
    finally:
        mgr.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())


