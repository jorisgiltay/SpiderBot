#!/usr/bin/env python
from typing import Dict, Tuple, List, Optional
import serial.tools.list_ports

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from STservo_sdk import PortHandler, sts, COMM_SUCCESS
from STservo_sdk.sts import (
    STS_ID,
    STS_BAUD_RATE,
    STS_MIN_ANGLE_LIMIT_L,
    STS_MIN_ANGLE_LIMIT_H,
    STS_MAX_ANGLE_LIMIT_L,
    STS_MAX_ANGLE_LIMIT_H,
    STS_CW_DEAD,
    STS_CCW_DEAD,
    STS_OFS_L,
    STS_OFS_H,
    STS_MODE,
    STS_TORQUE_ENABLE,
    STS_ACC,
    STS_GOAL_POSITION_L,
    STS_GOAL_POSITION_H,
    STS_GOAL_TIME_L,
    STS_GOAL_TIME_H,
    STS_GOAL_SPEED_L,
    STS_GOAL_SPEED_H,
    STS_LOCK,
    STS_PRESENT_POSITION_L,
    STS_PRESENT_POSITION_H,
    STS_PRESENT_SPEED_L,
    STS_PRESENT_SPEED_H,
    STS_PRESENT_LOAD_L,
    STS_PRESENT_LOAD_H,
    STS_PRESENT_VOLTAGE,
    STS_PRESENT_TEMPERATURE,
    STS_MOVING,
    STS_PRESENT_CURRENT_L,
    STS_PRESENT_CURRENT_H,
)


BaudRateMap: Dict[str, int] = {
    "1M": 0,
    "0_5M": 1,
    "250K": 2,
    "128K": 3,
    "115200": 4,
    "76800": 5,
    "57600": 6,
    "38400": 7,
}


class ServoManager:
    def __init__(self):
        self.port_handler: Optional[PortHandler] = None
        self.packet_handler: Optional[sts] = None
        self.is_connected: bool = False

    def list_serial_ports(self) -> List[str]:
        return [p.device for p in serial.tools.list_ports.comports()]

    def connect(self, port: str, baudrate: int) -> Tuple[bool, Optional[str]]:
        try:
            # Ensure any previous handle is fully closed before reconnect
            try:
                self.disconnect()
            except Exception:
                pass
            self.port_handler = PortHandler(port)
            if not self.port_handler.openPort():
                return False, "Failed to open port"
            if not self.port_handler.setBaudRate(baudrate):
                self.port_handler.closePort()
                return False, "Failed to set baudrate"
            self.packet_handler = sts(self.port_handler)
            self.is_connected = True
            return True, None
        except Exception as e:
            return False, str(e)

    def disconnect(self) -> None:
        # Mark disconnected first to stop new operations
        self.is_connected = False
        try:
            if self.port_handler:
                self.port_handler.closePort()
        finally:
            self.port_handler = None
            self.packet_handler = None

    def scan_ids(self, start_id: int = 1, end_id: int = 30) -> List[int]:
        found: List[int] = []
        if not self.is_connected or self.packet_handler is None:
            return found
        for sid in range(start_id, end_id + 1):
            try:
                _model, res, _ = self.packet_handler.ping(sid)  # type: ignore
                if res == COMM_SUCCESS:
                    found.append(sid)
            except Exception:
                # Ignore ping errors and continue
                pass
        return found

    def _read2(self, sid: int, addr_l: int) -> Tuple[int, int, int]:
        return self.packet_handler.read2ByteTxRx(sid, addr_l)  # type: ignore

    def _read1(self, sid: int, addr: int) -> Tuple[int, int, int]:
        return self.packet_handler.read1ByteTxRx(sid, addr)  # type: ignore

    def _write1(self, sid: int, addr: int, val: int) -> Tuple[int, int]:
        return self.packet_handler.write1ByteTxRx(sid, addr, val)  # type: ignore

    def _write2(self, sid: int, addr_l: int, val: int) -> Tuple[int, int]:
        return self.packet_handler.write2ByteTxRx(sid, addr_l, val)  # type: ignore

    def read_all_params(self, sid: int) -> Tuple[Dict, Optional[str]]:
        # EPROM R/W
        id_val, r1, e1 = self._read1(sid, STS_ID)
        baud, r2, e2 = self._read1(sid, STS_BAUD_RATE)
        min_lim, r3, e3 = self._read2(sid, STS_MIN_ANGLE_LIMIT_L)
        max_lim, r4, e4 = self._read2(sid, STS_MAX_ANGLE_LIMIT_L)
        cw_dead, r5, e5 = self._read1(sid, STS_CW_DEAD)
        ccw_dead, r6, e6 = self._read1(sid, STS_CCW_DEAD)
        ofs, r7, e7 = self._read2(sid, STS_OFS_L)
        mode, r8, e8 = self._read1(sid, STS_MODE)

        # SRAM R/W and R/O
        tq, r9, e9 = self._read1(sid, STS_TORQUE_ENABLE)
        acc, r10, e10 = self._read1(sid, STS_ACC)
        goal_pos, r11, e11 = self._read2(sid, STS_GOAL_POSITION_L)
        goal_time, r12, e12 = self._read2(sid, STS_GOAL_TIME_L)
        goal_spd, r13, e13 = self._read2(sid, STS_GOAL_SPEED_L)
        lock, r14, e14 = self._read1(sid, STS_LOCK)

        pres_pos, r15, e15 = self._read2(sid, STS_PRESENT_POSITION_L)
        pres_spd, r16, e16 = self._read2(sid, STS_PRESENT_SPEED_L)
        pres_load, r17, e17 = self._read2(sid, STS_PRESENT_LOAD_L)
        pres_volt, r18, e18 = self._read1(sid, STS_PRESENT_VOLTAGE)
        pres_temp, r19, e19 = self._read1(sid, STS_PRESENT_TEMPERATURE)
        moving, r20, e20 = self._read1(sid, STS_MOVING)
        pres_curr, r21, e21 = self._read2(sid, STS_PRESENT_CURRENT_L)

        results = [r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12,r13,r14,r15,r16,r17,r18,r19,r20,r21]
        errors = [e1,e2,e3,e4,e5,e6,e7,e8,e9,e10,e11,e12,e13,e14,e15,e16,e17,e18,e19,e20,e21]
        if any(r != COMM_SUCCESS for r in results) or any(e != 0 for e in errors):
            return {}, "Read error"

        return {
            "eprom": {
                "id": id_val,
                "baud": baud,
                "min_angle_limit": min_lim,
                "max_angle_limit": max_lim,
                "cw_dead": cw_dead,
                "ccw_dead": ccw_dead,
                "offset": ofs,
                "mode": mode,
            },
            "sram": {
                "torque_enable": tq,
                "acc": acc,
                "goal_position": goal_pos,
                "goal_time": goal_time,
                "goal_speed": goal_spd,
                "lock": lock,
            },
            "status": {
                "present_position": pres_pos,
                "present_speed": pres_spd,
                "present_load": pres_load,
                "present_voltage": pres_volt,
                "present_temperature": pres_temp,
                "moving": moving,
                "present_current": pres_curr,
            }
        }, None

    def write_params(self, sid: int, params: Dict) -> Tuple[bool, Optional[str]]:
        try:
            eprom = params.get("eprom", {})
            sram = params.get("sram", {})

            # If any EPROM keys are present, ensure torque is disabled and EPROM is unlocked
            eprom_keys = {"id","baud","min_angle_limit","max_angle_limit","cw_dead","ccw_dead","offset","mode"}
            will_write_eprom = any(k in eprom for k in eprom_keys)
            prev_torque = None
            prev_lock = None
            if will_write_eprom:
                prev_torque, rt, et = self._read1(sid, STS_TORQUE_ENABLE)
                prev_lock, rl, el = self._read1(sid, STS_LOCK)
                if rt == COMM_SUCCESS and et == 0 and prev_torque != 0:
                    self._write1(sid, STS_TORQUE_ENABLE, 0)
                if rl == COMM_SUCCESS and el == 0 and prev_lock != 0:
                    self._write1(sid, STS_LOCK, 0)

            # Write EPROM fields
            if "id" in eprom:
                res, err = self._write1(sid, STS_ID, int(eprom["id"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing ID"
                sid = int(eprom["id"])  # from now on, refer to new id

            if "baud" in eprom:
                # Changing baud will cut current connection; do it last among EPROM writes
                pass

            if "min_angle_limit" in eprom:
                res, err = self._write2(sid, STS_MIN_ANGLE_LIMIT_L, int(eprom["min_angle_limit"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing min angle"

            if "max_angle_limit" in eprom:
                res, err = self._write2(sid, STS_MAX_ANGLE_LIMIT_L, int(eprom["max_angle_limit"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing max angle"

            if "cw_dead" in eprom:
                res, err = self._write1(sid, STS_CW_DEAD, int(eprom["cw_dead"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing cw_dead"

            if "ccw_dead" in eprom:
                res, err = self._write1(sid, STS_CCW_DEAD, int(eprom["ccw_dead"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing ccw_dead"

            if "offset" in eprom:
                res, err = self._write2(sid, STS_OFS_L, int(eprom["offset"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing offset"

            if "mode" in eprom:
                res, err = self._write1(sid, STS_MODE, int(eprom["mode"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing mode"

            # Handle baud last (this may break communication, so don't try to restore states after this)
            if "baud" in eprom:
                res, err = self._write1(sid, STS_BAUD_RATE, int(eprom["baud"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing baud"
                # Cannot safely restore torque/lock after baud change without reconnect
                return True, None

            # Restore lock/torque if we changed them (for non-baud EPROM writes)
            if will_write_eprom:
                if prev_lock is not None and prev_lock != 0:
                    self._write1(sid, STS_LOCK, 1)
                if prev_torque is not None and prev_torque != 0:
                    self._write1(sid, STS_TORQUE_ENABLE, 1)

            # Write SRAM
            if "torque_enable" in sram:
                res, err = self._write1(sid, STS_TORQUE_ENABLE, int(bool(sram["torque_enable"])) )
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing torque"

            if "acc" in sram:
                res, err = self._write1(sid, STS_ACC, int(sram["acc"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing acc"

            if "goal_position" in sram:
                res, err = self._write2(sid, STS_GOAL_POSITION_L, int(sram["goal_position"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing goal position"

            if "goal_time" in sram:
                res, err = self._write2(sid, STS_GOAL_TIME_L, int(sram["goal_time"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing goal time"

            if "goal_speed" in sram:
                res, err = self._write2(sid, STS_GOAL_SPEED_L, int(sram["goal_speed"]))
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing goal speed"

            if "lock" in sram:
                val = 1 if bool(sram["lock"]) else 0
                res, err = self._write1(sid, STS_LOCK, val)
                if res != COMM_SUCCESS or err != 0:
                    return False, "Failed writing lock"

            return True, None
        except Exception as e:
            return False, str(e)

    def read_status(self, sid: int) -> Tuple[Dict, Optional[str]]:
        try:
            # Include goal position so UI can show setpoint vs actual
            goal_pos, r0, e0 = self._read2(sid, STS_GOAL_POSITION_L)
            pres_pos, r1, e1 = self._read2(sid, STS_PRESENT_POSITION_L)
            pres_spd, r2, e2 = self._read2(sid, STS_PRESENT_SPEED_L)
            pres_load, r3, e3 = self._read2(sid, STS_PRESENT_LOAD_L)
            pres_volt, r4, e4 = self._read1(sid, STS_PRESENT_VOLTAGE)
            pres_temp, r5, e5 = self._read1(sid, STS_PRESENT_TEMPERATURE)
            moving, r6, e6 = self._read1(sid, STS_MOVING)
            pres_curr, r7, e7 = self._read2(sid, STS_PRESENT_CURRENT_L)

            results = [r0,r1,r2,r3,r4,r5,r6,r7]
            errors = [e0,e1,e2,e3,e4,e5,e6,e7]
            if any(r != COMM_SUCCESS for r in results) or any(e != 0 for e in errors):
                return {}, "Read error"
            return {
                "goal_position": goal_pos,
                "present_position": pres_pos,
                "present_speed": pres_spd,
                "present_load": pres_load,
                "present_voltage": pres_volt,
                "present_temperature": pres_temp,
                "moving": moving,
                "present_current": pres_curr,
            }, None
        except Exception as e:
            return {}, str(e)

    def write_position(self, sid: int, position: int, speed: int, acc: int) -> Tuple[bool, Optional[str]]:
        try:
            # Ensure torque enabled
            self._write1(sid, STS_TORQUE_ENABLE, 1)
            # Stop any wheel motion first to avoid continued spinning
            try:
                self.packet_handler.WriteSpec(sid, 0, acc)  # type: ignore
            except Exception:
                pass
            # Switch to position mode
            try:
                # 0 = position mode
                self._write1(sid, STS_MODE, 0)
            except Exception:
                pass
            # Now write position command
            res, err = self.packet_handler.WritePosEx(sid, position, speed, acc)  # type: ignore
            if res != COMM_SUCCESS or err != 0:
                return False, "Write position failed"
            return True, None
        except Exception as e:
            return False, str(e)

    def write_wheel_speed(self, sid: int, speed: int, acc: int) -> Tuple[bool, Optional[str]]:
        try:
            # Ensure torque enabled
            self._write1(sid, STS_TORQUE_ENABLE, 1)
            # Put wheel mode and write speed
            res, err = self.packet_handler.WheelMode(sid)  # type: ignore
            if res != COMM_SUCCESS or err != 0:
                return False, "WheelMode failed"
            res, err = self.packet_handler.WriteSpec(sid, speed, acc)  # type: ignore
            if res != COMM_SUCCESS or err != 0:
                return False, "WriteSpec failed"
            return True, None
        except Exception as e:
            return False, str(e)

    def set_torque(self, sid: int, enable: bool) -> Tuple[bool, Optional[str]]:
        try:
            val = 1 if enable else 0
            res, err = self._write1(sid, STS_TORQUE_ENABLE, val)
            if res != COMM_SUCCESS or err != 0:
                return False, "Torque write failed"
            return True, None
        except Exception as e:
            return False, str(e)

    def set_lock(self, sid: int, lock: bool) -> Tuple[bool, Optional[str]]:
        try:
            val = 1 if lock else 0
            res, err = self._write1(sid, STS_LOCK, val)
            if res != COMM_SUCCESS or err != 0:
                return False, "Lock write failed"
            return True, None
        except Exception as e:
            return False, str(e)

    def change_id(self, sid: int, new_id: int) -> Tuple[bool, Optional[str]]:
        try:
            # check availability
            _, res_check, _ = self.packet_handler.ping(new_id)  # type: ignore
            if res_check == COMM_SUCCESS:
                return False, "New ID already in use"

            # Ensure torque disabled and EPROM unlocked
            prev_torque, rt, et = self._read1(sid, STS_TORQUE_ENABLE)
            prev_lock, rl, el = self._read1(sid, STS_LOCK)
            if rt == COMM_SUCCESS and et == 0 and prev_torque != 0:
                self._write1(sid, STS_TORQUE_ENABLE, 0)
            if rl == COMM_SUCCESS and el == 0 and prev_lock != 0:
                self._write1(sid, STS_LOCK, 0)

            # Write new ID
            res, err = self._write1(sid, STS_ID, new_id)
            if res != COMM_SUCCESS or err != 0:
                return False, "ID change failed"

            # Restore lock/torque using new ID
            if prev_lock is not None and prev_lock != 0:
                self._write1(new_id, STS_LOCK, 1)
            if prev_torque is not None and prev_torque != 0:
                self._write1(new_id, STS_TORQUE_ENABLE, 1)

            return True, None
        except Exception as e:
            return False, str(e)

    def change_baud(self, sid: int, baud_key: str) -> Tuple[bool, Optional[str]]:
        try:
            val = BaudRateMap[baud_key]
            # Ensure torque disabled and EPROM unlocked
            prev_torque, rt, et = self._read1(sid, STS_TORQUE_ENABLE)
            prev_lock, rl, el = self._read1(sid, STS_LOCK)
            if rt == COMM_SUCCESS and et == 0 and prev_torque != 0:
                self._write1(sid, STS_TORQUE_ENABLE, 0)
            if rl == COMM_SUCCESS and el == 0 and prev_lock != 0:
                self._write1(sid, STS_LOCK, 0)

            res, err = self._write1(sid, STS_BAUD_RATE, val)
            if res != COMM_SUCCESS or err != 0:
                return False, "Baud write failed"

            # Cannot restore states here because baud just changed; user must reconnect
            return True, None
        except Exception as e:
            return False, str(e)


