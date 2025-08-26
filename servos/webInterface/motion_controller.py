#!/usr/bin/env python
import os
import json
import time
import threading
from typing import Dict, Tuple, Optional


class MotionController:
    """
    Background motion controller for simple gait directions and height control.

    It uses a provided ServoManager instance and an external lock to serialize
    hardware access with other API handlers.
    """

    def __init__(self, servo_manager, manager_lock: threading.Lock):
        self.servo_manager = servo_manager
        self.manager_lock = manager_lock
        self.thread: Optional[threading.Thread] = None
        self.stop_event = threading.Event()
        self.mode: Optional[str] = None  # 'forward' | 'backward' | 'left' | 'right'
        self.params: Dict = {}
        self.ranges = self._load_servo_ranges().get("servos", {})

    # ---------- Public API ----------
    def is_running(self) -> bool:
        return self.thread is not None and self.thread.is_alive()

    def current_mode(self) -> Optional[str]:
        return self.mode if self.is_running() else None

    def start_gait(self, mode: str, params: Optional[Dict] = None) -> Tuple[bool, Optional[str]]:
        # Modes:
        #  - forward, backward
        #  - left, right         -> crab left/right (strafe)
        #  - pivot_left, pivot_right -> on-spot rotation
        if mode not in ("forward", "backward", "left", "right", "pivot_left", "pivot_right"):
            return False, "Invalid mode"
        if not self.servo_manager.is_connected:
            return False, "Not connected"
        if self.is_running():
            # Stop existing first
            self.stop()
            # small wait to ensure previous thread exits
            time.sleep(0.05)

        self.mode = mode
        self.params = params or {}
        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run_loop, name=f"gait-{mode}", daemon=True)
        self.thread.start()
        return True, None

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        self.thread = None
        self.mode = None

    def set_height(self, height_mm: float, speed: int = 2400, acc: int = 50) -> Tuple[bool, Optional[str]]:
        if not self.servo_manager.is_connected:
            return False, "Not connected"
        # Keep hips at mid, adjust all knees to height
        knee_ids = [2, 4, 6, 8]
        hip_ids = [1, 3, 5, 7]
        with self.manager_lock:
            # hips to mid
            for hip_id in hip_ids:
                mid = self._servo_mid(hip_id)
                self.servo_manager.write_position(hip_id, mid, speed, acc)
            # knees to mapped height
            for knee_id in knee_ids:
                tgt = self._knee_for_height(knee_id, height_mm)
                self.servo_manager.write_position(knee_id, tgt, speed, acc)
        return True, None

    # ---------- Internals ----------
    def _load_servo_ranges(self) -> Dict:
        # ranges file lives at ../servo_ranges.json relative to this file
        here = os.path.dirname(__file__)
        path = os.path.abspath(os.path.join(here, "..", "servo_ranges.json"))
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def _clamp_ticks(self, val: int, smin: int, smax: int) -> int:
        return max(smin, min(smax, val))

    def _servo_mid(self, sid: int) -> int:
        s = self.ranges.get(f"servo{sid}", {})
        try:
            return self._clamp_ticks(int(s["mid"]), int(s["min"]), int(s["max"]))
        except Exception:
            return 2048

    def _knee_for_height(self, sid: int, height_mm: float) -> int:
        s = self.ranges.get(f"servo{sid}", {})
        try:
            base_mid = int(s["mid"]) ; smin = int(s["min"]) ; smax = int(s["max"]) 
        except Exception:
            base_mid, smin, smax = 2048, 0, 4095
        # mapping: 0mm -> max_ticks (low), 60mm -> base_mid (high)
        max_height_mm = 60.0
        h = max(0.0, min(max_height_mm, float(height_mm)))
        t = h / max_height_mm
        target = int(round((1.0 - t) * smax + t * base_mid))
        return self._clamp_ticks(target, smin, smax)

    def _hip_bounds(self, sid: int) -> Tuple[int, int, int]:
        s = self.ranges.get(f"servo{sid}", {})
        try:
            return int(s["min"]), int(s["mid"]), int(s["max"])  # (min, mid, max)
        except Exception:
            return 0, 2048, 4095

    def _side_sign_for_hip(self, hip_id: int) -> int:
        # Invert right hips (3 and 7) to align directions across sides for forward/back/pivot
        return -1 if hip_id in (3, 7) else 1

    def _frontrear_sign_for_hip(self, hip_id: int) -> int:
        # Invert rear hips (5 and 7) to align directions across front vs rear for crab (sideways)
        return -1 if hip_id in (5, 7) else 1

    def _hip_step_ticks(self, hip_id: int, step_mm: float, hip_mm_range: float) -> int:
        hmin, hmid, hmax = self._hip_bounds(hip_id)
        hip_range_ticks = max(1, (hmax - hmin))
        if hip_mm_range <= 0.0:
            return 0
        ticks_per_mm = (hip_range_ticks / 2.0) / hip_mm_range
        return int(round(step_mm * ticks_per_mm))

    def _write_leg(self, hip_id: int, knee_id: int, hip_pos: int, knee_pos: int, speed: int, acc: int) -> None:
        self.servo_manager.write_position(hip_id, hip_pos, speed, acc)
        self.servo_manager.write_position(knee_id, knee_pos, speed, acc)

    def _run_loop(self) -> None:
        # Parameters with defaults
        speed = int(self.params.get("speed", 2400))
        acc = int(self.params.get("acc", 50))
        base_height = float(self.params.get("height", 20.0))
        lift_mm = float(self.params.get("lift", 8.0))
        step_mm = float(self.params.get("step", 18.0))
        stride_time = max(0.2, float(self.params.get("stride_time", 0.8)))
        hip_mm_range = float(self.params.get("hip_mm_range", 35.0))
        # Step magnitude (no turn bias by default)
        left_bias = 1.0
        right_bias = 1.0

        # Servos mapping
        FL = (1, 2)
        FR = (3, 4)
        RL = (5, 6)
        RR = (7, 8)

        # Neutral pose
        neutral = {
            1: self._servo_mid(1),
            2: self._knee_for_height(2, base_height),
            3: self._servo_mid(3),
            4: self._knee_for_height(4, base_height),
            5: self._servo_mid(5),
            6: self._knee_for_height(6, base_height),
            7: self._servo_mid(7),
            8: self._knee_for_height(8, base_height),
        }

        # Precompute step ticks per hip with bias per side
        raw_steps = {
            1: self._hip_step_ticks(1, step_mm, hip_mm_range),
            3: self._hip_step_ticks(3, step_mm, hip_mm_range),
            5: self._hip_step_ticks(5, step_mm, hip_mm_range),
            7: self._hip_step_ticks(7, step_mm, hip_mm_range),
        }
        hip_steps = {
            1: int(round(raw_steps[1] * left_bias)),
            5: int(round(raw_steps[5] * left_bias)),
            3: int(round(raw_steps[3] * right_bias)),
            7: int(round(raw_steps[7] * right_bias)),
        }

        step_T = stride_time / 4.0

        # Per-leg forward/backward mapping
        # forward=True means stance moves foot backward relative to body
        if self.mode == "forward":
            per_leg_forward = {1: True, 3: True, 5: True, 7: True}
        elif self.mode == "backward":
            per_leg_forward = {1: False, 3: False, 5: False, 7: False}
        elif self.mode == "pivot_left":
            # Left side opposite of right side to rotate left
            per_leg_forward = {1: False, 5: False, 3: True, 7: True}
        elif self.mode == "pivot_right":
            per_leg_forward = {1: True, 5: True, 3: False, 7: False}
        elif self.mode == "left":  # crab left
            # Use uniform direction but flip sign by front/rear axis in kinematics
            per_leg_forward = {1: False, 3: False, 5: False, 7: False}
        else:  # "right" crab right
            per_leg_forward = {1: True, 3: True, 5: True, 7: True}

        def stance_phase(hip_id: int, knee_id: int, forward: bool) -> None:
            hmin, hmid, hmax = self._hip_bounds(hip_id)
            base = hmid
            step = hip_steps[hip_id]
            # Choose sign scheme: forward/back/pivot use left/right inversion; crab uses front/rear inversion
            if self.mode in ("left", "right"):
                sign = self._frontrear_sign_for_hip(hip_id)
            else:
                sign = self._side_sign_for_hip(hip_id)
            signed_step = sign * step
            if forward:
                target = self._clamp_ticks(base - signed_step, hmin, hmax)
            else:
                target = self._clamp_ticks(base + signed_step, hmin, hmax)
            self._write_leg(hip_id, knee_id, target, self._knee_for_height(knee_id, base_height), speed, acc)

        def swing_phase(hip_id: int, knee_id: int, forward: bool) -> None:
            base_knee = self._knee_for_height(knee_id, base_height)
            lift_knee = self._knee_for_height(knee_id, max(0.0, base_height - lift_mm))
            hmin, hmid, hmax = self._hip_bounds(hip_id)
            base = hmid
            step = hip_steps[hip_id]
            if self.mode in ("left", "right"):
                sign = self._frontrear_sign_for_hip(hip_id)
            else:
                sign = self._side_sign_for_hip(hip_id)
            signed_step = sign * step
            if forward:
                target = self._clamp_ticks(base + signed_step, hmin, hmax)
            else:
                target = self._clamp_ticks(base - signed_step, hmin, hmax)
            self._write_leg(hip_id, knee_id, target, lift_knee, speed, acc)
            time.sleep(step_T * 0.5)
            # Lower
            self.servo_manager.write_position(knee_id, base_knee, speed, acc)
            time.sleep(max(0.0, step_T * 0.5))

        # Move to neutral
        with self.manager_lock:
            for sid, pos in neutral.items():
                self.servo_manager.write_position(sid, pos, speed, acc)
        time.sleep(0.4)

        # Sequence
        leg_sequence = [FL, RR, FR, RL]
        try:
            while not self.stop_event.is_set():
                for swing_hip, swing_knee in leg_sequence:
                    if self.stop_event.is_set():
                        break
                    with self.manager_lock:
                        for (hip_id, knee_id) in [FL, RR, FR, RL]:
                            if hip_id != swing_hip:
                                stance_phase(hip_id, knee_id, forward=per_leg_forward[hip_id])
                        swing_phase(swing_hip, swing_knee, forward=per_leg_forward[swing_hip])
        finally:
            # Return to neutral at the end
            with self.manager_lock:
                for sid, pos in neutral.items():
                    self.servo_manager.write_position(sid, pos, speed, acc)
            time.sleep(0.3)


