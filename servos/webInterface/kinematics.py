from __future__ import annotations

import math
from typing import Dict, Tuple
import os
import sys

# Ensure repo root is importable so we can reuse the exact formula from delta_y_plot.py
_here = os.path.dirname(__file__)
sys.path.append(os.path.join(_here, "..", ".."))
from delta_y_plot import height_of_theta as dy_height_of_theta  # type: ignore


def compute_alignment_offset(r1: float, off1_deg: float, r2: float, off2_deg: float, switch_deg: float) -> float:
    y1 = r1 * math.sin(math.radians(switch_deg - off1_deg))
    y2 = r2 * math.sin(math.radians(switch_deg - off2_deg))
    return y1 - y2


def raw_height_of_theta(theta_deg: float, r1: float, off1_deg: float, r2: float, off2_deg: float, switch_deg: float, align: bool = True) -> float:
    # Delegate to the canonical function defined in delta_y_plot.py
    return dy_height_of_theta(
        theta_deg=theta_deg,
        r1=r1,
        theta_off1_deg=off1_deg,
        r2=r2,
        theta_off2_deg=off2_deg,
        theta_switch_deg=switch_deg,
        align_switch=align,
    )


def height_of_theta(theta_deg: float, r1: float, off1_deg: float, r2: float, off2_deg: float, switch_deg: float, align: bool = True) -> float:
    # Use the exact same function as the plotting script
    return raw_height_of_theta(theta_deg, r1, off1_deg, r2, off2_deg, switch_deg, align)


def height_of_theta_calibrated(
    theta_deg: float,
    r1: float,
    off1_deg: float,
    r2: float,
    off2_deg: float,
    switch_deg: float,
    cal_theta0_deg: float,
    cal_height0_mm: float,
    cal_theta1_deg: float,
    cal_height1_mm: float,
    align: bool = True,
) -> float:
    r0 = raw_height_of_theta(cal_theta0_deg, r1, off1_deg, r2, off2_deg, switch_deg, align)
    r1h = raw_height_of_theta(cal_theta1_deg, r1, off1_deg, r2, off2_deg, switch_deg, align)
    # Solve y = a*raw + b for two calibration points
    denom = (r1h - r0) if (r1h - r0) != 0 else 1e-9
    a = (cal_height1_mm - cal_height0_mm) / denom
    b = cal_height0_mm - a * r0
    raw = raw_height_of_theta(theta_deg, r1, off1_deg, r2, off2_deg, switch_deg, align)
    return a * raw + b


def theta_from_height(y: float, r1: float, off1_deg: float, r2: float, off2_deg: float, switch_deg: float, prefer_region: str = "auto", align: bool = True) -> float:
    """Invert y(theta). prefer_region in {"A","B","auto"}. Returns theta in degrees.

    Region A: theta > switch_deg uses r1, off1
    Region B: theta <= switch_deg uses r2, off2 (+ alignment c)
    """
    # Try both regions and pick the theta producing y closest to target, respecting region constraint
    c_align = compute_alignment_offset(r1, off1_deg, r2, off2_deg, switch_deg) if align else 0.0

    candidates: Dict[str, float] = {}

    # Region A inversion
    x = max(-1.0, min(1.0, y / r1)) if r1 != 0 else 0.0
    try:
        a = math.degrees(math.asin(x))
        theta_a = off1_deg + a
        candidates["A"] = theta_a
    except ValueError:
        pass

    # Region B inversion
    x2 = (y - c_align) / r2 if r2 != 0 else 0.0
    x2 = max(-1.0, min(1.0, x2))
    try:
        b = math.degrees(math.asin(x2))
        theta_b = off2_deg + b
        candidates["B"] = theta_b
    except ValueError:
        pass

    # Filter by preference
    if prefer_region == "A" and "A" in candidates:
        return candidates["A"]
    if prefer_region == "B" and "B" in candidates:
        return candidates["B"]

    # Auto: choose whose forward height is nearest to target and respects region limits
    best_theta = None
    best_err = float("inf")
    for region, theta in candidates.items():
        if region == "A" and not (theta > switch_deg):
            continue
        if region == "B" and not (theta <= switch_deg):
            continue
        y_hat = height_of_theta(theta, r1, off1_deg, r2, off2_deg, switch_deg, align)
        err = abs(y_hat - y)
        if err < best_err:
            best_err = err
            best_theta = theta
    if best_theta is not None:
        return best_theta

    # Fallback: return Region A if available else Region B
    return candidates.get("A", candidates.get("B", switch_deg))


def theta_from_height_calibrated(
    y_mm: float,
    r1: float,
    off1_deg: float,
    r2: float,
    off2_deg: float,
    switch_deg: float,
    cal_theta0_deg: float,
    cal_height0_mm: float,
    cal_theta1_deg: float,
    cal_height1_mm: float,
    prefer_region: str = "auto",
    align: bool = True,
) -> float:
    # Compute calibration mapping a,b such that y = a*raw + b
    r0 = raw_height_of_theta(cal_theta0_deg, r1, off1_deg, r2, off2_deg, switch_deg, align)
    r1h = raw_height_of_theta(cal_theta1_deg, r1, off1_deg, r2, off2_deg, switch_deg, align)
    denom = (r1h - r0) if (r1h - r0) != 0 else 1e-9
    a = (cal_height1_mm - cal_height0_mm) / denom
    b = cal_height0_mm - a * r0
    # Invert calibration then invert raw model
    raw_target = (y_mm - b) / a
    # We can reuse theta_from_height by passing y in place of raw y with r1/r2/offs
    return theta_from_height(raw_target, r1, off1_deg, r2, off2_deg, switch_deg, prefer_region, align)


def heights_from_roll_pitch(roll_deg: float, pitch_deg: float, span_roll_mm: float, span_pitch_mm: float, ref_height_mm: float, scale: float = 1.0) -> Dict[str, float]:
    hx = span_roll_mm / 2.0
    hy = span_pitch_mm / 2.0
    tanR = math.tan(math.radians(roll_deg))
    tanP = math.tan(math.radians(pitch_deg))
    zFL = ref_height_mm + scale * (-tanR * hx + tanP * hy)
    zFR = ref_height_mm + scale * ( tanR * hx + tanP * hy)
    zRL = ref_height_mm + scale * (-tanR * hx - tanP * hy)
    zRR = ref_height_mm + scale * ( tanR * hx - tanP * hy)
    return {"FL": zFL, "FR": zFR, "RL": zRL, "RR": zRR}


