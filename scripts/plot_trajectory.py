#!/usr/bin/env python3
"""
plot_trajectory.py — parse gz topic pose log and plot glider trajectory.

Usage:
    # First capture pose data:
    gz topic -e -t /model/tuba_glider/pose > /tmp/pose_log.txt

    # Then plot:
    python3 scripts/plot_trajectory.py --input /tmp/pose_log.txt

Output:
    trajectory.png  — z vs time + x vs time side-by-side
"""

import argparse
import re
import sys
import os

try:
    import matplotlib.pyplot as plt
    import numpy as np
except ImportError:
    sys.exit("Install matplotlib and numpy: pip install matplotlib numpy")


def parse_pose_log(path: str):
    """Extract sim_time, x, z from gz topic pose text output."""
    times = []
    xs = []
    zs = []

    time_re  = re.compile(r"sec:\s*(\d+).*?nsec:\s*(\d+)", re.DOTALL)
    pos_x_re = re.compile(r"x:\s*([-\d.]+)")
    pos_z_re = re.compile(r"z:\s*([-\d.]+)")

    with open(path) as f:
        content = f.read()

    # Split on "---" message separators if present, else work line by line
    blocks = re.split(r"---+\n?", content)

    for block in blocks:
        tm = time_re.search(block)
        px = pos_x_re.search(block)
        pz = pos_z_re.search(block)
        if tm and px and pz:
            t = int(tm.group(1)) + int(tm.group(2)) * 1e-9
            x = float(px.group(1))
            z = float(pz.group(1))
            times.append(t)
            xs.append(x)
            zs.append(z)

    if not times:
        sys.exit(f"No pose data parsed from {path}. "
                 "Ensure the file was captured with: "
                 "gz topic -e -t /model/tuba_glider/pose")

    # Normalise time to start at 0
    t0 = times[0]
    times = [t - t0 for t in times]

    return np.array(times), np.array(xs), np.array(zs)


def main():
    parser = argparse.ArgumentParser(description="Plot TUBA glider trajectory")
    parser.add_argument("--input", required=True, help="Path to pose log file")
    parser.add_argument("--output", default="trajectory.png",
                        help="Output PNG path (default: trajectory.png)")
    args = parser.parse_args()

    times, xs, zs = parse_pose_log(args.input)

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("TUBA Glider Trajectory", fontsize=14)

    # Left: depth over time
    ax = axes[0]
    ax.plot(times, -zs, color="steelblue", linewidth=1.2)
    ax.set_xlabel("Simulation time (s)")
    ax.set_ylabel("Depth (m, positive = below surface)")
    ax.invert_yaxis()
    ax.set_title("Depth vs Time")
    ax.grid(True, alpha=0.4)

    # Right: XZ trajectory (side view of glide path)
    ax = axes[1]
    ax.plot(xs, zs, color="coral", linewidth=1.2)
    ax.set_xlabel("Forward distance X (m)")
    ax.set_ylabel("Altitude Z (m, positive = up)")
    ax.set_title("Glide Path (side view)")
    ax.grid(True, alpha=0.4)

    # Annotate glide angle if enough data
    if len(xs) > 2:
        dx = xs[-1] - xs[0]
        dz = zs[-1] - zs[0]
        if dx > 0.01:
            angle = abs(np.degrees(np.arctan2(dz, dx)))
            axes[1].annotate(
                f"Net glide angle: {angle:.1f}°",
                xy=(0.05, 0.05), xycoords="axes fraction",
                fontsize=10, color="darkred"
            )

    plt.tight_layout()
    out_path = os.path.abspath(args.output)
    plt.savefig(out_path, dpi=150)
    print(f"Saved: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
