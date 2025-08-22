#!/usr/bin/env python
import threading
from flask import Flask, jsonify, request, render_template, send_from_directory
from servo_manager import ServoManager, BaudRateMap


app = Flask(__name__, static_folder="static", template_folder="templates")
manager_lock = threading.Lock()
servo_manager = ServoManager()


@app.route("/")
def index():
    return render_template("index.html")


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


if __name__ == "__main__":
    # Bind to all interfaces so you can reach it from other devices on the network
    app.run(host="0.0.0.0", port=5000, debug=False)


