#!/usr/bin/env python3
"""
Compute and plot:

    delta_y = | R1 * sin(theta - theta_off1 + delta_theta) - R1 * sin(theta - theta_off1) |

All angles are in degrees.

Defaults:
  - theta_base = 15 deg (default)
  - theta_switch = -30 deg (default)
  - theta_end = -67.5 deg (default)
  - We sweep delta_theta from 0 to (theta_end - theta_base)
  - Piecewise parameters:
      Region A (theta > switch): R1=77.816, theta_off1=31 deg
      Region B (theta ≤ switch): R2=85.788, theta_off2=22.5 deg

Single continuous height formula y(theta):
  Let C_align = R1*sin(theta_switch - theta_off1) - R2*sin(theta_switch - theta_off2)
  Then
    y(theta) = { R1*sin(theta - theta_off1),                 if theta > theta_switch
               { R2*sin(theta - theta_off2) + C_align,       if theta ≤ theta_switch

Usage examples:
  - Show interactive plot sweeping delta_theta with two regions:
      python delta_y_plot.py --theta-base 15 --theta-switch -30 --theta-end -67.5 \
        --r1 77.816 --theta-off1 31 --r2 85.788 --theta-off2 22.5

  - Save plot to PNG without showing a window:
      python delta_y_plot.py --theta-base 15 --theta-end -30 --save plot.png --no-show
"""

from __future__ import annotations

import argparse
import math
import sys
from typing import List


def try_import_matplotlib():
    try:
        import matplotlib.pyplot as plt  # type: ignore
        return plt
    except Exception as exc:  # pragma: no cover - best-effort helper
        print(
            "This script requires matplotlib. Install it with: pip install matplotlib",
            file=sys.stderr,
        )
        raise


def generate_degree_range(start_deg: float, end_deg: float, step_deg: float) -> List[float]:
    """Generate a list of degree values including the end, handling up/down ranges.

    Ensures we step in the correct direction and include the end value if it falls
    exactly on a step; otherwise the last step stops just before crossing the end.
    """
    if step_deg == 0:
        raise ValueError("step must be non-zero")

    step = step_deg if end_deg >= start_deg else -abs(step_deg)
    values: List[float] = []

    current = start_deg
    if step > 0:
        while current <= end_deg + 1e-12:
            values.append(round(current, 10))
            current += step
    else:
        while current >= end_deg - 1e-12:
            values.append(round(current, 10))
            current += step

    # Ensure exact end inclusion when within numeric tolerance
    if (values and abs(values[-1] - end_deg) > 1e-9) and (
        (step > 0 and values[-1] < end_deg) or (step < 0 and values[-1] > end_deg)
    ):
        values.append(end_deg)

    return values


def compute_alignment_offset(
    r1: float,
    theta_off1_deg: float,
    r2: float,
    theta_off2_deg: float,
    theta_switch_deg: float,
) -> float:
    """Compute vertical offset to align Region B to Region A at theta_switch."""
    y1 = r1 * math.sin(math.radians(theta_switch_deg - theta_off1_deg))
    y2 = r2 * math.sin(math.radians(theta_switch_deg - theta_off2_deg))
    return y1 - y2


def height_of_theta(
    theta_deg: float,
    r1: float,
    theta_off1_deg: float,
    r2: float,
    theta_off2_deg: float,
    theta_switch_deg: float,
    align_switch: bool = True,
) -> float:
    """Return height y(theta) using piecewise parameters with optional alignment."""
    if theta_deg > theta_switch_deg:
        return r1 * math.sin(math.radians(theta_deg - theta_off1_deg))
    c_align = 0.0
    if align_switch:
        c_align = compute_alignment_offset(r1, theta_off1_deg, r2, theta_off2_deg, theta_switch_deg)
    return r2 * math.sin(math.radians(theta_deg - theta_off2_deg)) + c_align


def compute_y_shift_values(
    r: float,
    theta_off_deg: float,
    theta_base_deg: float,
    delta_theta_deg_values: List[float],
) -> List[float]:
    """Compute absolute y values for the shifted term at theta = theta_base + delta."""
    values: List[float] = []
    base_minus_off_deg = theta_base_deg - theta_off_deg
    for dtheta_deg in delta_theta_deg_values:
        values.append(r * math.sin(math.radians(base_minus_off_deg + dtheta_deg)))
    return values


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Plot delta_y vs theta (degrees, pre-offset)")
    parser.add_argument(
        "--theta-base",
        type=float,
        default=15.0,
        help="Base theta in degrees (starting position, default: 15)",
    )
    parser.add_argument(
        "--theta-end",
        type=float,
        default=-67.5,
        help="End theta in degrees (default: -67.5). Defines total delta sweep.",
    )
    parser.add_argument(
        "--theta-switch",
        type=float,
        default=-30.0,
        help="Switch theta where parameters change (default: -30)",
    )
    parser.add_argument(
        "--delta-step",
        type=float,
        default=0.5,
        help="Step size for delta_theta in degrees (default: 0.5)",
    )
    parser.add_argument(
        "--r1",
        type=float,
        default=77.816,
        help="R1 value (default: 77.816)",
    )
    parser.add_argument(
        "--theta-off1",
        type=float,
        default=31.0,
        help="Theta offset in degrees for region A (default: 31)",
    )
    parser.add_argument(
        "--r2",
        type=float,
        default=85.788,
        help="R2 value used after switch (default: 85.788)",
    )
    parser.add_argument(
        "--theta-off2",
        type=float,
        default=22.5,
        help="Theta offset in degrees for region B (default: 22.5)",
    )
    parser.add_argument(
        "--plot-height",
        action="store_true",
        help="Also plot absolute height y(theta) below the delta plot",
    )
    parser.add_argument(
        "--no-align-switch",
        action="store_true",
        help="Do not vertically align region B to region A at the switch",
    )
    parser.add_argument(
        "--save",
        type=str,
        default="",
        help="Optional path to save the figure (e.g., output.png)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not display an interactive window",
    )
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()

    total_delta = args.theta_end - args.theta_base
    delta_theta_values = generate_degree_range(0.0, total_delta, args.delta_step)
    plt = try_import_matplotlib()

    if args.plot_height:
        fig, (ax, ax_h) = plt.subplots(2, 1, figsize=(8, 8), sharex=True, height_ratios=[2, 1])
    else:
        fig, ax = plt.subplots(figsize=(8, 5))
        ax_h = None
    theta_axis_values = [args.theta_base + d for d in delta_theta_values]

    # Split by region using the switch threshold
    region_a_deltas: List[float] = [d for d, th in zip(delta_theta_values, theta_axis_values) if th > args.theta_switch]
    region_b_deltas: List[float] = [d for d, th in zip(delta_theta_values, theta_axis_values) if th <= args.theta_switch]

    # Fixed baseline at theta_base using Region A parameters
    y_base = args.r1 * math.sin(math.radians(args.theta_base - args.theta_off1))

    if region_a_deltas:
        region_a_thetas = [args.theta_base + d for d in region_a_deltas]
        y_shift_a = compute_y_shift_values(
            r=args.r1,
            theta_off_deg=args.theta_off1,
            theta_base_deg=args.theta_base,
            delta_theta_deg_values=region_a_deltas,
        )
        region_a_values = [abs(y - y_base) for y in y_shift_a]
        ax.plot(
            region_a_thetas,
            region_a_values,
            marker="o",
            markersize=3,
            linewidth=1.5,
            label=f"Region A: R={args.r1}, off={args.theta_off1}°",
        )

    if region_b_deltas:
        region_b_thetas = [args.theta_base + d for d in region_b_deltas]
        y_shift_b = compute_y_shift_values(
            r=args.r2,
            theta_off_deg=args.theta_off2,
            theta_base_deg=args.theta_base,
            delta_theta_deg_values=region_b_deltas,
        )
        region_b_values = [abs(y - y_base) for y in y_shift_b]
        ax.plot(
            region_b_thetas,
            region_b_values,
            marker="o",
            markersize=3,
            linewidth=1.5,
            label=f"Region B: R={args.r2}, off={args.theta_off2}°",
        )
    ax.grid(True, which="both", linestyle=":", linewidth=0.8)
    ax.set_xlabel("theta (deg, pre-offset)")
    ax.set_ylabel("delta_y (same units as R)")
    ax.set_title("delta_y vs theta (pre-offset)")
    ax.legend(loc="best")

    # Optional absolute height plot
    if ax_h is not None:
        align_switch = not args.no_align_switch
        y_values = [
            height_of_theta(
                theta_deg=th,
                r1=args.r1,
                theta_off1_deg=args.theta_off1,
                r2=args.r2,
                theta_off2_deg=args.theta_off2,
                theta_switch_deg=args.theta_switch,
                align_switch=align_switch,
            )
            for th in theta_axis_values
        ]
        ax_h.plot(theta_axis_values, y_values, color="C2", linewidth=1.5, label="y(theta)")
        ax_h.grid(True, which="both", linestyle=":", linewidth=0.8)
        ax_h.set_ylabel("y (height)")
        ax_h.set_xlabel("theta (deg, pre-offset)")
        ax_h.legend(loc="best")

    if args.save:
        fig.tight_layout()
        fig.savefig(args.save, dpi=150)
        print(f"Saved figure to: {args.save}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()


