#!/usr/bin/env python3
"""
glider_mission.py — surface / dive / climb mission controller for my_glider_v2

Run while gz sim is running tuba_ocean.sdf (bash scripts/run_sim.sh):

    python3 scripts/glider_mission.py            # repeat cycles forever
    python3 scripts/glider_mission.py --cycles 1 --log /tmp/mission.csv

Mission (defaults):
  SURFACE     float ~70-75 deg nose-down, antenna out of the water
  DIVE_STEEP  bladder to dive volume, battery fully forward (~75 deg) until 2 m
  DIVE        pitch 30 deg nose-down until 10 m
  CLIMB       bladder to climb volume, pitch 30 deg nose-up until 1 m
  SURFACING   battery fully forward, bladder neutral until pitched past 60 deg
              nose-down, then surface volume until the tail breaks the surface

Depth is measured at the hull centre. Pitch is positive nose-down (gz convention).
Pitch is held by the battery (mass shifter) using a feedforward from the hull
trim (pitch ~= atan(CG offset / 6 mm), see model.sdf) plus a PI correction.
"""

import argparse
import math
import os
import sys
import threading
import time

# Must match the sim's GZ_IP (scripts/run_sim.sh uses loopback), otherwise
# gz-transport discovery may never find the sim.
os.environ.setdefault("GZ_IP", "127.0.0.1")

try:
    from gz.transport import Node
    from gz.msgs.double_pb2 import Double
    from gz.msgs.pose_v_pb2 import Pose_V
except ImportError as e:
    sys.exit(f"Could not import gz Python bindings: {e}")

MODEL = "my_glider_v2"
WORLD = "tuba_ocean"
POSE_TOPIC = f"/world/{WORLD}/dynamic_pose/info"
BUOY_TOPIC = f"/model/{MODEL}/buoyancy_engine"
BATT_TOPIC = f"/{MODEL}/battery_joint_pos"

SURFACE_Z = 20.0          # world water surface height
HULL_CENTRE_X = -0.5387   # model frame, see hull_seg_* in model.sdf
TAIL_END_X = -1.1887      # aft end of the hull cylinder

# Battery joint range and trim (from model.sdf): battery position for a given
# pitch is ~ BATT_LEVEL + BATT_PER_TAN * tan(pitch).
BATT_MIN, BATT_MAX = -0.05, 0.14
BATT_LEVEL = -0.0195
BATT_PER_TAN = 0.0428
BATT_RATE = 0.03          # m/s, mass-shifter drive speed


def pitch_from_quat(q):
    s = 2.0 * (q.w * q.y - q.z * q.x)
    return math.asin(max(-1.0, min(1.0, s)))


class Glider:
    def __init__(self):
        self.node = Node()
        self.lock = threading.Lock()
        self.state = None  # (sim_time, x, z, pitch)
        self.buoy_pub = self.node.advertise(BUOY_TOPIC, Double)
        self.batt_pub = self.node.advertise(BATT_TOPIC, Double)
        if not self.node.subscribe(Pose_V, POSE_TOPIC, self._on_pose):
            sys.exit(f"Failed to subscribe to {POSE_TOPIC}")

    def _on_pose(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == MODEL:
                with self.lock:
                    self.state = (t, p.position.x, p.position.z,
                                  pitch_from_quat(p.orientation))

    def read(self):
        with self.lock:
            return self.state

    def set_volume(self, v):
        m = Double()
        m.data = v
        self.buoy_pub.publish(m)

    def set_battery(self, x):
        m = Double()
        m.data = x
        self.batt_pub.publish(m)


def batt_feedforward(pitch):
    return BATT_LEVEL + BATT_PER_TAN * math.tan(pitch)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cycles", type=int, default=0, help="0 = run forever")
    ap.add_argument("--surface-time", type=float, default=20.0, help="s at surface")
    ap.add_argument("--steep-depth", type=float, default=2.0)
    ap.add_argument("--dive-depth", type=float, default=10.0)
    ap.add_argument("--climb-depth", type=float, default=1.0,
                    help="depth at which to pitch nose-down for surfacing")
    ap.add_argument("--glide-pitch", type=float, default=30.0, help="deg")
    ap.add_argument("--surface-volume", type=float, default=0.0006, help="m^3")
    ap.add_argument("--neutral-volume", type=float, default=0.0005, help="m^3")
    ap.add_argument("--surfacing-pitch", type=float, default=60.0,
                    help="deg nose-down before adding surface reserve")
    ap.add_argument("--dive-volume", type=float, default=0.00035, help="m^3")
    ap.add_argument("--climb-volume", type=float, default=0.00065, help="m^3")
    ap.add_argument("--kp", type=float, default=0.06, help="battery m per rad")
    ap.add_argument("--ki", type=float, default=0.006, help="battery m per rad*s")
    ap.add_argument("--log", help="CSV log file")
    args = ap.parse_args()

    g = Glider()
    print(f"[mission] waiting for glider pose on {POSE_TOPIC} "
          f"(GZ_IP={os.environ['GZ_IP']}) ...", flush=True)
    while g.read() is None:
        time.sleep(0.1)

    glide = math.radians(args.glide_pitch)
    log = open(args.log, "w") if args.log else None
    if log:
        log.write("t,state,x,depth,pitch_deg,target_deg,battery,volume\n")

    state, t_state, cycle = "SURFACE", g.read()[0], 0
    batt_cmd, integ, last_t, last_print = BATT_MAX, 0.0, None, -1e9
    max_depth = 0.0

    def enter(new, t):
        nonlocal state, t_state, integ
        print(f"[mission] t={t:7.1f}s  {state} -> {new}", flush=True)
        state, t_state, integ = new, t, 0.0

    while True:
        t, x, z, pitch = g.read()
        if last_t is not None and t <= last_t:
            time.sleep(0.02)
            continue
        dt = 0.0 if last_t is None else min(t - last_t, 0.5)
        last_t = t

        depth = SURFACE_Z - (z - HULL_CENTRE_X * math.sin(pitch))
        tail_z = z - TAIL_END_X * math.sin(pitch)
        max_depth = max(max_depth, depth)

        # ---- state machine -------------------------------------------------
        if state == "SURFACE":
            volume, target = args.surface_volume, None
            if t - t_state >= args.surface_time:
                if args.cycles and cycle >= args.cycles:
                    print(f"[mission] done after {cycle} cycle(s)", flush=True)
                    break
                cycle += 1
                max_depth = 0.0
                enter("DIVE_STEEP", t)
        elif state == "DIVE_STEEP":
            volume, target = args.dive_volume, None
            if depth >= args.steep_depth:
                enter("DIVE", t)
        elif state == "DIVE":
            volume, target = args.dive_volume, glide
            if depth >= args.dive_depth:
                enter("CLIMB", t)
        elif state == "CLIMB":
            volume, target = args.climb_volume, -glide
            if depth <= args.climb_depth:
                print(f"[mission] cycle {cycle}: max depth {max_depth:.2f} m",
                      flush=True)
                enter("SURFACING", t)
        elif state == "SURFACING":
            # Hover at neutral while the battery pitches the nose down, then
            # add the small surface reserve so it rises tail (antenna) first.
            steep = pitch >= math.radians(args.surfacing_pitch)
            volume = args.surface_volume if steep else args.neutral_volume
            target = None
            if tail_z >= SURFACE_Z:
                enter("SURFACE", t)

        # ---- pitch control via battery -------------------------------------
        if target is None:
            want = BATT_MAX           # fully forward: steepest nose-down
        else:
            err = target - pitch
            integ = max(-0.5, min(0.5, integ + err * dt))
            want = batt_feedforward(target) + args.kp * err + args.ki * integ
        want = max(BATT_MIN, min(BATT_MAX, want))
        step = BATT_RATE * dt
        batt_cmd += max(-step, min(step, want - batt_cmd))

        g.set_volume(volume)
        g.set_battery(batt_cmd)

        tgt_deg = "" if target is None else f"{math.degrees(target):.0f}"
        if log:
            log.write(f"{t:.2f},{state},{x:.3f},{depth:.3f},"
                      f"{math.degrees(pitch):.2f},{tgt_deg},{batt_cmd:.4f},{volume}\n")
        if t - last_print >= 2.0:
            last_print = t
            print(f"  t={t:7.1f}s {state:10s} depth={depth:6.2f} m "
                  f"pitch={math.degrees(pitch):+6.1f} deg (target {tgt_deg or 'max'}) "
                  f"batt={batt_cmd:+.3f} vol={volume*1000:.2f} L "
                  f"tail_z={tail_z - SURFACE_Z:+.3f} m", flush=True)
        time.sleep(0.02)

    if log:
        log.close()


if __name__ == "__main__":
    main()
