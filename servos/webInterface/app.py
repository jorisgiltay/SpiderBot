#!/usr/bin/env python
import threading
import os
import json
from flask import Flask, jsonify, request, render_template, send_from_directory
from servo_manager import ServoManager, BaudRateMap
from motion_controller import MotionController
from kinematics import heights_from_roll_pitch, theta_from_height_calibrated, height_of_theta_calibrated


_SERVO_RANGES = None


def _load_servo_ranges():
    global _SERVO_RANGES
    if _SERVO_RANGES is not None:
        return _SERVO_RANGES
    # Path to servo_ranges.json (one level up from this file)
    here = os.path.dirname(__file__)
    path = os.path.join(here, "..", "servo_ranges.json")
    try:
        with open(path, "r", encoding="utf-8") as f:
            _SERVO_RANGES = json.load(f)
    except Exception:
        _SERVO_RANGES = {"servos": {}}
    return _SERVO_RANGES


def _deg_to_ticks_for_servo(sid: int, theta_deg: float) -> int:
    """Map theta in deg to servo ticks using per-servo min/max.

    Assumes:
      min ticks corresponds to -90 deg, max ticks corresponds to +15 deg.
    Clamps theta to [-90, 15].
    """
    data = _load_servo_ranges().get("servos", {})
    entry = data.get(f"servo{sid}") or {}
    tmin = int(entry.get("min", 0))
    tmax = int(entry.get("max", 4095))
    # Clamp angle
    lo, hi = -90.0, 15.0
    th = max(lo, min(hi, float(theta_deg)))
    # Linear map th in [lo,hi] -> [tmin,tmax]
    ratio = (th - lo) / (hi - lo) if hi != lo else 0.0
    ticks = tmin + ratio * (tmax - tmin)
    return int(round(ticks))


app = Flask(__name__, static_folder="static", template_folder="templates")
manager_lock = threading.Lock()
servo_manager = ServoManager()
motion_controller = MotionController(servo_manager, manager_lock)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/controls")
def controls():
    return render_template("controls.html")


@app.route("/attitude")
def attitude():
    return render_template("attitude.html")


@app.get("/api/ports")
def list_ports():
    return jsonify({"ports": servo_manager.list_serial_ports()})


@app.post("/api/connect")
def connect():
    payload = request.get_json(silent=True) or {}
    port = payload.get("port")
    baudrate = int(payload.get("baudrate", 1000000))
    if not port:
        return jsonify({"ok": False, "error": "Missing 'port'"}), 400
    with manager_lock:
        ok, err = servo_manager.connect(port, baudrate)
    return jsonify({"ok": ok, "error": err})


@app.post("/api/disconnect")
def disconnect():
    with manager_lock:
        servo_manager.disconnect()
    return jsonify({"ok": True})


@app.get("/api/scan")
def scan():
    try:
        start_id = int(request.args.get("start", 1))
        end_id = int(request.args.get("end", 30))
    except Exception:
        return jsonify({"ok": False, "error": "Invalid id range"}), 400
    with manager_lock:
        if not servo_manager.is_connected:
            return jsonify({"ok": False, "error": "Not connected"}), 400
        ids = servo_manager.scan_ids(start_id, end_id)
    return jsonify({"ok": True, "ids": ids})


@app.get("/api/params/<int:servo_id>")
def get_params(servo_id: int):
    with manager_lock:
        if not servo_manager.is_connected:
            return jsonify({"ok": False, "error": "Not connected"}), 400
        data, err = servo_manager.read_all_params(servo_id)
    if err:
        return jsonify({"ok": False, "error": err}), 400
    return jsonify({"ok": True, "params": data, "baudRateMap": BaudRateMap})


@app.post("/api/params/<int:servo_id>")
def set_params(servo_id: int):
    payload = request.get_json(silent=True) or {}
    with manager_lock:
        if not servo_manager.is_connected:
            return jsonify({"ok": False, "error": "Not connected"}), 400
        ok, err = servo_manager.write_params(servo_id, payload)
    return jsonify({"ok": ok, "error": err})


@app.get("/api/status/<int:servo_id>")
def status(servo_id: int):
    with manager_lock:
        if not servo_manager.is_connected:
            return jsonify({"ok": False, "error": "Not connected"}), 400
        data, err = servo_manager.read_status(servo_id)
    if err:
        return jsonify({"ok": False, "error": err}), 400
    return jsonify({"ok": True, "status": data})


@app.post("/api/command/<int:servo_id>/position")
def command_position(servo_id: int):
    p = request.get_json(silent=True) or {}
    position = int(p.get("position", 0))
    speed = int(p.get("speed", 2400))
    acc = int(p.get("acc", 50))
    with manager_lock:
        ok, err = servo_manager.write_position(servo_id, position, speed, acc)
    return jsonify({"ok": ok, "error": err})


@app.post("/api/command/<int:servo_id>/wheel")
def command_wheel(servo_id: int):
    p = request.get_json(silent=True) or {}
    speed = int(p.get("speed", 0))
    acc = int(p.get("acc", 50))
    with manager_lock:
        ok, err = servo_manager.write_wheel_speed(servo_id, speed, acc)
    return jsonify({"ok": ok, "error": err})


@app.post("/api/command/<int:servo_id>/torque")
def command_torque(servo_id: int):
    p = request.get_json(silent=True) or {}
    enable = bool(p.get("enable", True))
    with manager_lock:
        ok, err = servo_manager.set_torque(servo_id, enable)
    return jsonify({"ok": ok, "error": err})


@app.post("/api/command/<int:servo_id>/lock")
def command_lock(servo_id: int):
    p = request.get_json(silent=True) or {}
    lock = bool(p.get("lock", True))
    with manager_lock:
        ok, err = servo_manager.set_lock(servo_id, lock)
    return jsonify({"ok": ok, "error": err})


@app.post("/api/command/<int:servo_id>/change_id")
def command_change_id(servo_id: int):
    p = request.get_json(silent=True) or {}
    new_id = p.get("new_id")
    if new_id is None:
        return jsonify({"ok": False, "error": "Missing new_id"}), 400
    with manager_lock:
        ok, err = servo_manager.change_id(servo_id, int(new_id))
    return jsonify({"ok": ok, "error": err})


@app.post("/api/command/<int:servo_id>/baud")
def command_baud(servo_id: int):
    p = request.get_json(silent=True) or {}
    baud_key = p.get("baud_key")
    if baud_key not in BaudRateMap:
        return jsonify({"ok": False, "error": "Invalid baud_key"}), 400
    with manager_lock:
        ok, err = servo_manager.change_baud(servo_id, baud_key)
    return jsonify({"ok": ok, "error": err})


# ---------------- Motion control endpoints ----------------
@app.post("/api/motion/start")
def motion_start():
    p = request.get_json(silent=True) or {}
    mode = p.get("mode")
    params = p.get("params") or {}
    ok, err = motion_controller.start_gait(str(mode), params)
    return jsonify({"ok": ok, "error": err, "mode": motion_controller.current_mode()})


@app.post("/api/motion/stop")
def motion_stop():
    motion_controller.stop()
    return jsonify({"ok": True})


@app.post("/api/motion/height")
def motion_height():
    p = request.get_json(silent=True) or {}
    try:
        height = float(p.get("height", 20.0))
        speed = int(p.get("speed", 2400))
        acc = int(p.get("acc", 50))
    except Exception:
        return jsonify({"ok": False, "error": "Invalid parameters"}), 400
    ok, err = motion_controller.set_height(height, speed, acc)
    return jsonify({"ok": ok, "error": err})


# ---------------- Attitude computation/apply ----------------
@app.post("/api/attitude/angles")
def attitude_angles():
    p = request.get_json(silent=True) or {}
    roll = float(p.get("roll", 0.0))
    pitch = float(p.get("pitch", 0.0))
    span_roll = float(p.get("span_roll", 93.0))
    span_pitch = float(p.get("span_pitch", 80.0))
    ref_height = float(p.get("ref_height", 35.0))
    scale = float(p.get("scale", 1.0))
    r1 = float(p.get("r1", 77.816))
    off1 = float(p.get("off1", 31.0))
    r2 = float(p.get("r2", 85.788))
    off2 = float(p.get("off2", 22.5))
    switch = float(p.get("switch", -30.0))

    heights = heights_from_roll_pitch(roll, pitch, span_roll, span_pitch, ref_height, scale)
    # Calibration: y(15°)=0 mm, y(-67.5°)=64 mm
    cal0_theta, cal0_h = 15.0, 0.0
    cal1_theta, cal1_h = -67.5, 64.0
    thetas = {k: theta_from_height_calibrated(v, r1, off1, r2, off2, switch, cal0_theta, cal0_h, cal1_theta, cal1_h, prefer_region="auto", align=True) for k, v in heights.items()}
    return jsonify({"ok": True, "heights": heights, "thetas": thetas})


@app.post("/api/attitude/apply")
def attitude_apply():
    p = request.get_json(silent=True) or {}
    mapping = p.get("mapping") or {"FL": 2, "FR": 4, "RL": 6, "RR": 8}
    speed = int(p.get("speed", 2400))
    acc = int(p.get("acc", 50))
    bias_ticks = float(p.get("bias_ticks", 0.0))  # optional additive bias per-servo (global)
    invert = bool(p.get("invert", False))

    # Reuse computation endpoint logic
    resp = attitude_angles()
    data = resp.get_json()  # type: ignore
    if not data.get("ok"):
        return resp
    thetas = data.get("thetas", {})

    results = {}
    with manager_lock:
        is_conn = servo_manager.is_connected
        for corner, theta_deg in thetas.items():
            tdeg = -float(theta_deg) if invert else float(theta_deg)
            sid = mapping.get(corner)
            if sid is None:
                results[corner] = {"ok": False, "error": "Missing servo id"}
                continue
            ticks = _deg_to_ticks_for_servo(int(sid), tdeg)
            ticks = int(round(ticks + bias_ticks))
            if is_conn:
                ok, err = servo_manager.write_position(int(sid), ticks, speed, acc)
                results[corner] = {"ok": ok, "error": err, "ticks": ticks, "theta_deg": tdeg}
            else:
                results[corner] = {"ok": False, "error": "Not connected (preview)", "ticks": ticks, "theta_deg": tdeg}
    return jsonify({"ok": is_conn, "connected": is_conn, "results": results, "thetas": thetas, "bias_ticks": bias_ticks})


if __name__ == "__main__":
    # Bind to all interfaces so you can reach it from other devices on the network
    app.run(host="0.0.0.0", port=5000, debug=False)


