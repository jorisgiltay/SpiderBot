#!/usr/bin/env python
import threading
import os
import json
from flask import Flask, jsonify, request, render_template, send_from_directory
from typing import Optional, Tuple, Dict
import time
import struct
try:
    import serial  # type: ignore
except Exception:
    serial = None  # type: ignore
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

# MSP (IMU) connection state
msp_lock = threading.Lock()
msp_serial = None


# ---------------- MSP helpers ----------------
MSP_ATTITUDE = 108   # Attitude data (roll, pitch, yaw)
MSP_ANALOG = 110     # Analog readings (vbat, amperage, rssi, etc.)


def _msp_build_request(command: int, payload: bytes = b"") -> bytes:
    header = b"$M<"
    size = len(payload)
    size_b = size.to_bytes(1, "little")
    cmd_b = command.to_bytes(1, "little")
    checksum = size ^ command
    for b in payload:
        checksum ^= b
    csum_b = checksum.to_bytes(1, "little")
    return header + size_b + cmd_b + payload + csum_b


def _msp_read_response(ser, timeout_s: float = 0.2) -> bytes:
    buf = bytearray()
    start_time = time.time()
    while time.time() - start_time < timeout_s:
        buf += ser.read(ser.in_waiting or 1)
        idx = buf.find(b"$M>")
        if idx != -1 and len(buf) >= idx + 5:
            size = buf[idx + 3]
            total_len = 3 + 1 + 1 + size + 1
            if len(buf) >= idx + total_len:
                return bytes(buf[idx: idx + total_len])
    return bytes(buf)


def _msp_parse_attitude(response: bytes):
    if len(response) < 5:
        return None
    size = response[3]
    if size != 6:
        return None
    payload = response[5:5 + size]
    if len(payload) != 6:
        return None
    roll_raw, pitch_raw, yaw_raw = struct.unpack('<3h', payload)
    return roll_raw, pitch_raw, yaw_raw


def _msp_parse_analog(response: bytes):
    """Parse MSP_ANALOG. Returns dict with at least 'vbat' (0.1V units) when available."""
    if len(response) < 5:
        return None
    size = response[3]
    # Betaflight MSPv1 typically returns size >= 7
    if size < 1:
        return None
    payload = response[5:5 + size]
    if len(payload) < 1:
        return None
    # First byte is VBAT in 0.1V units (total battery voltage)
    vbat_raw = payload[0]
    out = {"vbat": int(vbat_raw)}
    # Optionally parse amperage (2 bytes) and mAh drawn (2 bytes) if present
    try:
        if len(payload) >= 5:
            # amperage in 0.01A units (int16), mAh drawn (uint16)
            amperage_raw = struct.unpack('<h', payload[1:3])[0]
            mah_drawn = struct.unpack('<H', payload[3:5])[0]
            out.update({"amperage_cA": int(amperage_raw), "mah": int(mah_drawn)})
    except Exception:
        pass
    return out


# ---------------- Roll PID (new page) ----------------
class RollPID:
    def __init__(self) -> None:
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        # Gains and runtime
        self.kp = 2.0
        self.ki = 0.03
        self.kd = 0.32
        self.rate_hz = 30.0
        self.limit_deg = 30.0
        self.integral_limit = 100.0  # deg*s, anti-windup clamp
        # Setpoint and base
        self.set_roll = 0.0
        self.set_pitch = 0.0
        self.base_roll = 0.0
        self.base_pitch = 0.0
        # Geometry/scaling
        self.span_roll = 93.0
        self.span_pitch = 80.0
        self.ref_height = 34.0
        self.scale = 2.0
        # Auto-level stabilization (tunable)
        self.auto_level_gain = 15.0       # multiply inverse measured roll
        self.auto_level_tau = 0.25        # seconds, low-pass on measured roll
        self.cmd_slew_deg_per_s = 10.0    # limit cmd_roll change rate
        self.cmd_speed = 3600             # servo speed (ticks/s)
        self.cmd_acc = 80                 # servo acc (0..255)
        self.deriv_tau = 0.12             # seconds, low-pass on derivative
        self.control_pitch = True         # enable pitch control
        # State
        self.integral = 0.0
        self.prev_err = 0.0
        self.filtered_roll = 0.0
        self.prev_cmd_roll = 0.0
        self._deriv_filt = 0.0
        # Pitch state
        self.integral_p = 0.0
        self.prev_err_p = 0.0
        self.filtered_pitch = 0.0
        self.prev_cmd_pitch = 0.0
        self._deriv_filt_p = 0.0
        # fixed action sign (no UI invert)
        self.invert_action = False
        self.last: Dict = {}

    def is_running(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def configure(self, p: Dict) -> None:
        # Clamp gains to non-negative to avoid accidental sign flips
        self.kp = max(0.0, float(p.get("kp", self.kp)))
        self.ki = max(0.0, float(p.get("ki", self.ki)))
        self.kd = max(0.0, float(p.get("kd", self.kd)))
        self.rate_hz = max(1.0, float(p.get("rate_hz", self.rate_hz)))
        self.limit_deg = max(0.1, float(p.get("limit_deg", self.limit_deg)))
        self.integral_limit = float(p.get("integral_limit", self.integral_limit))
        self.span_roll = float(p.get("span_roll", self.span_roll))
        self.span_pitch = float(p.get("span_pitch", self.span_pitch))
        self.ref_height = float(p.get("ref_height", self.ref_height))
        self.scale = float(p.get("scale", self.scale))
        # Optional stabilization params
        if "auto_level_gain" in p:
            self.auto_level_gain = max(0.0, float(p.get("auto_level_gain", self.auto_level_gain)))
        if "auto_level_tau" in p:
            self.auto_level_tau = max(0.0, float(p.get("auto_level_tau", self.auto_level_tau)))
        if "cmd_slew_deg_per_s" in p:
            self.cmd_slew_deg_per_s = max(0.0, float(p.get("cmd_slew_deg_per_s", self.cmd_slew_deg_per_s)))
        if "cmd_speed" in p:
            self.cmd_speed = max(0, int(p.get("cmd_speed", self.cmd_speed)))
        if "cmd_acc" in p:
            self.cmd_acc = max(0, int(p.get("cmd_acc", self.cmd_acc)))
        if "deriv_tau" in p:
            self.deriv_tau = max(0.0, float(p.get("deriv_tau", self.deriv_tau)))
        if "enable_pitch" in p:
            self.control_pitch = bool(p.get("enable_pitch", self.control_pitch))
        # invert_action removed from UI; ignore if provided

    def set_targets(self, p: Dict) -> None:
        self.set_roll = float(p.get("set_roll", self.set_roll))
        self.set_pitch = float(p.get("set_pitch", self.set_pitch))
        self.base_roll = float(p.get("base_roll", self.base_roll))
        self.base_pitch = float(p.get("base_pitch", self.base_pitch))

    def start(self, p: Dict) -> Tuple[bool, Optional[str]]:
        if self.is_running():
            self.stop()
            time.sleep(0.05)
        self.configure(p)
        self.integral = 0.0
        self.prev_err = 0.0
        self.filtered_roll = 0.0
        self.prev_cmd_roll = 0.0
        self._deriv_filt = 0.0
        self.integral_p = 0.0
        self.prev_err_p = 0.0
        self.filtered_pitch = 0.0
        self.prev_cmd_pitch = 0.0
        self._deriv_filt_p = 0.0
        # reset state
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, name="pid-roll", daemon=True)
        self.thread.start()
        return True, None

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.3)
        self.thread = None

    def status(self) -> Dict:
        return {
            "running": self.is_running(),
            "kp": self.kp, "ki": self.ki, "kd": self.kd,
            "rate_hz": self.rate_hz, "limit_deg": self.limit_deg,
            "set_roll": self.set_roll,
            "base_roll": self.base_roll, "base_pitch": self.base_pitch,
            "span_roll": self.span_roll, "span_pitch": self.span_pitch,
            "ref_height": self.ref_height, "scale": self.scale,
            "last": self.last,
        }

    def _run(self) -> None:
        dt = 1.0 / self.rate_hz
        while not self.stop_event.is_set():
            # Read IMU
            with msp_lock:
                if msp_serial is None:
                    self.last = {"error": "MSP not connected"}
                    time.sleep(dt)
                    continue
                try:
                    msp_serial.reset_input_buffer()  # type: ignore
                    pkt = _msp_build_request(MSP_ATTITUDE)
                    msp_serial.write(pkt)  # type: ignore
                    time.sleep(0.005)
                    resp = _msp_read_response(msp_serial)  # type: ignore
                    att = _msp_parse_attitude(resp)
                except Exception as e:
                    self.last = {"error": str(e)}
                    att = None
            if not att:
                time.sleep(dt)
                continue
            roll_raw, pitch_raw, _ = att
            meas_roll = roll_raw / 10.0
            meas_pitch = pitch_raw / 10.0
            # Low-pass filter measured roll for auto-level feedforward
            if self.auto_level_tau > 0:
                alpha = (1.0 / self.rate_hz) / (self.auto_level_tau + (1.0 / self.rate_hz))
                self.filtered_roll = (1.0 - alpha) * self.filtered_roll + alpha * meas_roll
            else:
                self.filtered_roll = meas_roll
            # Low-pass filter pitch similarly
            if self.auto_level_tau > 0:
                alpha = (1.0 / self.rate_hz) / (self.auto_level_tau + (1.0 / self.rate_hz))
                self.filtered_pitch = (1.0 - alpha) * self.filtered_pitch + alpha * meas_pitch
            else:
                self.filtered_pitch = meas_pitch
            # Error in IMU convention: target - measured
            err = self.set_roll - meas_roll
            # PID
            self.integral += err * dt
            # Anti-windup
            if self.integral_limit > 0:
                self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
            raw_deriv = (err - self.prev_err) / dt
            if self.deriv_tau > 0:
                alpha_d = dt / (self.deriv_tau + dt)
                self._deriv_filt = (1.0 - alpha_d) * self._deriv_filt + alpha_d * raw_deriv
                deriv = self._deriv_filt
            else:
                deriv = raw_deriv
            u = self.kp*err + self.ki*self.integral + self.kd*deriv
            self.prev_err = err
            # Limit
            u = max(-self.limit_deg, min(self.limit_deg, u))
            # fixed sign convention: negative feedback

            # Compose commands with setpoint feedforward (match manual slider mapping)
            # Add auto-level feedforward (inverse of measured), scaled and filtered (keep 1.2 kinematic scaling)
            auto_sp = -self.auto_level_gain * self.filtered_roll
            raw_cmd_roll = 1.2*(self.base_roll + self.set_roll + auto_sp + u)
            # Slew-limit command to avoid sudden jumps
            if self.cmd_slew_deg_per_s > 0:
                max_delta = self.cmd_slew_deg_per_s * (1.0 / self.rate_hz)
                delta = raw_cmd_roll - self.prev_cmd_roll
                if delta > max_delta:
                    cmd_roll = self.prev_cmd_roll + max_delta
                elif delta < -max_delta:
                    cmd_roll = self.prev_cmd_roll - max_delta
                else:
                    cmd_roll = raw_cmd_roll
            else:
                cmd_roll = raw_cmd_roll
            self.prev_cmd_roll = cmd_roll
            # Pitch control (optional)
            if self.control_pitch:
                err_p = self.set_pitch - meas_pitch
                self.integral_p += err_p * dt
                if self.integral_limit > 0:
                    self.integral_p = max(-self.integral_limit, min(self.integral_limit, self.integral_p))
                raw_deriv_p = (err_p - self.prev_err_p) / dt
                if self.deriv_tau > 0:
                    alpha_dp = dt / (self.deriv_tau + dt)
                    self._deriv_filt_p = (1.0 - alpha_dp) * self._deriv_filt_p + alpha_dp * raw_deriv_p
                    deriv_p = self._deriv_filt_p
                else:
                    deriv_p = raw_deriv_p
                u_p = self.kp*err_p + self.ki*self.integral_p + self.kd*deriv_p
                self.prev_err_p = err_p
                u_p = max(-self.limit_deg, min(self.limit_deg, u_p))
                auto_sp_p = -self.auto_level_gain * self.filtered_pitch
                raw_cmd_pitch = 1.2*(self.base_pitch + self.set_pitch + auto_sp_p + u_p)
                if self.cmd_slew_deg_per_s > 0:
                    max_delta_p = self.cmd_slew_deg_per_s * (1.0 / self.rate_hz)
                    delta_p = raw_cmd_pitch - self.prev_cmd_pitch
                    if delta_p > max_delta_p:
                        cmd_pitch = self.prev_cmd_pitch + max_delta_p
                    elif delta_p < -max_delta_p:
                        cmd_pitch = self.prev_cmd_pitch - max_delta_p
                    else:
                        cmd_pitch = raw_cmd_pitch
                else:
                    cmd_pitch = raw_cmd_pitch
                self.prev_cmd_pitch = cmd_pitch
            else:
                auto_sp_p = 0.0
                u_p = 0.0
                cmd_pitch = self.base_pitch + self.set_pitch

            # (auto sign/gain calibration removed)

            # Apply via existing path
            try:
                # Ensure plant polarity: drive measured toward setpoint
                applied_roll = -cmd_roll
                applied_pitch = -cmd_pitch
                _ = _apply_attitude_internal(applied_roll, applied_pitch, {
                    "span_roll": self.span_roll,
                    "span_pitch": self.span_pitch,
                    "ref_height": self.ref_height,
                    "scale": self.scale,
                    "r1": 77.816, "off1": 31.0, "r2": 85.788, "off2": 22.5, "switch": -30.0,
                    "invert": False, "bias_ticks": 0.0,
                    "mapping": {"FL": 2, "FR": 4, "RL": 6, "RR": 8},
                    "speed": int(self.cmd_speed), "acc": int(self.cmd_acc),
                })
            except Exception:
                pass

            self.last = {
                "meas_roll": meas_roll,
                "meas_pitch": meas_pitch,
                "filtered_roll": self.filtered_roll,
                "filtered_pitch": self.filtered_pitch,
                "err": err,
                "u": u,
                "auto_sp": auto_sp,
                "cmd_roll": cmd_roll,
                "err_pitch": self.set_pitch - meas_pitch,
                "u_pitch": u_p,
                "auto_sp_pitch": auto_sp_p,
                "cmd_pitch": cmd_pitch,
                "applied_roll": applied_roll,
                "applied_pitch": applied_pitch,
            }
            time.sleep(dt)


roll_pid = RollPID()





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


@app.get("/api/servo/status")
def servo_status():
    with manager_lock:
        ok = bool(servo_manager.is_connected)
    return jsonify({"ok": ok})


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
    ref_height = float(p.get("ref_height", 34.0))
    scale = float(p.get("scale", 2.0))
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
        # Set hips (odd IDs) to mid like MotionController.set_height does
        if is_conn:
            try:
                for hip_id in (1, 3, 5, 7):
                    mid = motion_controller._servo_mid(hip_id)  # type: ignore
                    servo_manager.write_position(hip_id, mid, speed, acc)
            except Exception:
                pass
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


# ---------------- MSP endpoints ----------------
@app.post("/api/msp/connect")
def msp_connect():
    if serial is None:
        return jsonify({"ok": False, "error": "pyserial not available"}), 500
    p = request.get_json(silent=True) or {}
    port = p.get("port")
    baud = int(p.get("baud", 115200))
    if not port:
        return jsonify({"ok": False, "error": "Missing 'port'"}), 400
    global msp_serial
    with msp_lock:
        try:
            if msp_serial:
                try:
                    msp_serial.close()
                except Exception:
                    pass
            msp_serial = serial.Serial(port, baud, timeout=0.1)  # type: ignore
            return jsonify({"ok": True, "port": port, "baud": baud})
        except Exception as e:
            msp_serial = None
            return jsonify({"ok": False, "error": str(e)}), 400


@app.post("/api/msp/disconnect")
def msp_disconnect():
    global msp_serial
    with msp_lock:
        try:
            if msp_serial:
                msp_serial.close()
        finally:
            msp_serial = None
    return jsonify({"ok": True})


@app.get("/api/msp/status")
def msp_status():
    with msp_lock:
        ok = msp_serial is not None
    return jsonify({"ok": ok})


@app.get("/api/msp/attitude")
def msp_attitude():
    with msp_lock:
        if msp_serial is None:
            return jsonify({"ok": False, "error": "MSP not connected"}), 400
        try:
            msp_serial.reset_input_buffer()  # type: ignore
            pkt = _msp_build_request(MSP_ATTITUDE)
            msp_serial.write(pkt)  # type: ignore
            time.sleep(0.01)
            resp = _msp_read_response(msp_serial)  # type: ignore
            att = _msp_parse_attitude(resp)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 500
    if not att:
        return jsonify({"ok": False, "error": "No data"}), 500
    roll_raw, pitch_raw, yaw_raw = att
    return jsonify({
        "ok": True,
        "roll": roll_raw / 10.0,
        "pitch": pitch_raw / 10.0,
        "yaw": yaw_raw / 10.0,
    })


@app.get("/api/msp/battery")
def msp_battery():
    """Return total battery voltage (V) from MSP_ANALOG if available."""
    with msp_lock:
        if msp_serial is None:
            return jsonify({"ok": False, "error": "MSP not connected"}), 400
        try:
            msp_serial.reset_input_buffer()  # type: ignore
            pkt = _msp_build_request(MSP_ANALOG)
            msp_serial.write(pkt)  # type: ignore
            time.sleep(0.01)
            resp = _msp_read_response(msp_serial)  # type: ignore
            analog = _msp_parse_analog(resp)
        except Exception as e:
            return jsonify({"ok": False, "error": str(e)}), 500
    if not analog or "vbat" not in analog:
        return jsonify({"ok": False, "error": "No data"}), 500
    vbat_tenths = analog.get("vbat", 0)
    voltage = float(vbat_tenths) / 10.0
    out = {"ok": True, "voltage": voltage}
    if "amperage_cA" in analog:
        out["current"] = analog["amperage_cA"] / 100.0
    if "mah" in analog:
        out["mah"] = analog["mah"]
    return jsonify(out)


# ---------------- PID Roll endpoints ----------------
@app.post("/pid/start")
def pid_roll_start():
    p = request.get_json(silent=True) or {}
    ok, err = roll_pid.start(p)
    return jsonify({"ok": ok, "error": err})


@app.post("/pid/stop")
def pid_roll_stop():
    roll_pid.stop()
    return jsonify({"ok": True})


@app.post("/pid/config")
def pid_roll_config():
    p = request.get_json(silent=True) or {}
    roll_pid.configure(p)
    return jsonify({"ok": True})


@app.post("/pid/setpoint")
def pid_roll_setpoint():
    p = request.get_json(silent=True) or {}
    roll_pid.set_targets(p)
    return jsonify({"ok": True})


@app.get("/pid/status")
def pid_roll_status():
    return jsonify({"ok": True, "status": roll_pid.status()})


@app.route("/pid")
def pid_page():
    return render_template("pid.html")


def _apply_attitude_internal(roll: float, pitch: float, params: dict) -> dict:
    """Compute thetas and optionally write to servos. Returns result dict per-corner."""
    span_roll = float(params.get("span_roll", 93.0))
    span_pitch = float(params.get("span_pitch", 80.0))
    ref_height = float(params.get("ref_height", 34.0))
    scale = float(params.get("scale", 2.0))
    r1 = float(params.get("r1", 77.816))
    off1 = float(params.get("off1", 31.0))
    r2 = float(params.get("r2", 85.788))
    off2 = float(params.get("off2", 22.5))
    switch = float(params.get("switch", -30.0))
    invert = bool(params.get("invert", False))
    bias_ticks = float(params.get("bias_ticks", 0.0))
    mapping = params.get("mapping") or {"FL": 2, "FR": 4, "RL": 6, "RR": 8}
    speed = int(params.get("speed", 2400))
    acc = int(params.get("acc", 50))

    heights = heights_from_roll_pitch(roll, pitch, span_roll, span_pitch, ref_height, scale)
    # Calibration: y(15°)=0 mm, y(-67.5°)=64 mm
    cal0_theta, cal0_h = 15.0, 0.0
    cal1_theta, cal1_h = -67.5, 64.0
    thetas = {k: theta_from_height_calibrated(v, r1, off1, r2, off2, switch, cal0_theta, cal0_h, cal1_theta, cal1_h, prefer_region="auto", align=True) for k, v in heights.items()}

    out = {}
    with manager_lock:
        is_conn = servo_manager.is_connected
        # Set hips to mid first, like MotionController.set_height
        if is_conn:
            try:
                for hip_id in (1, 3, 5, 7):
                    mid = motion_controller._servo_mid(hip_id)  # type: ignore
                    servo_manager.write_position(hip_id, mid, speed, acc)
            except Exception:
                pass
        for corner, theta_deg in thetas.items():
            tdeg = -float(theta_deg) if invert else float(theta_deg)
            sid = mapping.get(corner)
            ticks = _deg_to_ticks_for_servo(int(sid), tdeg) if sid is not None else None
            if ticks is not None:
                ticks = int(round(ticks + bias_ticks))
            if is_conn and sid is not None and ticks is not None:
                ok, err = servo_manager.write_position(int(sid), int(ticks), speed, acc)
                out[corner] = {"ok": ok, "error": err, "ticks": ticks, "theta_deg": tdeg}
            else:
                out[corner] = {"ok": False, "error": "Not connected (preview)" if not is_conn else "Missing sid/ticks", "ticks": ticks, "theta_deg": tdeg}
    return {"thetas": thetas, "results": out, "heights": heights}



if __name__ == "__main__":
    # Bind to all interfaces so you can reach it from other devices on the network
    app.run(host="0.0.0.0", port=5000, debug=False)


