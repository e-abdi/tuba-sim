#!/usr/bin/env python3
"""
test_sil.py — end-to-end check of the software-in-the-loop stack

Starts (unless told they are already running) a headless Gazebo server,
the bridge and the native firmware, then drives the firmware's menus the
way an operator would and checks that the glider in Gazebo reacts:

  1. hardware test -> I2C scan: all five sensors present
  2. pump +6 s, pitch +12 s: glider floats ~70 deg nose-down, GPS fix, compass
  3. parameters for a 5 m dive, deploy: dive, climb, surface, stop on ENTER

    python3 scripts/test_sil.py                       # everything, ~6 min
    python3 scripts/test_sil.py --no-mission          # steps 1-2 only, ~1 min
    python3 scripts/test_sil.py --attach              # gz + bridge already running

Needs: python3-pexpect, bridge built (scripts/build_bridge.sh), firmware
built (tuba-firmware/sil_build.sh). Uses its own GZ_PARTITION so it never
touches a Gazebo you have open.
"""
import argparse
import os
import re
import signal
import subprocess
import sys
import time

import pexpect

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FW_DEFAULT = os.path.expanduser("~/zephyrproject/tuba/build_sil/zephyr/zephyr.exe")


def log(msg):
    print(f"[test] {time.strftime('%H:%M:%S')} {msg}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--firmware", default=FW_DEFAULT, help="native firmware executable")
    ap.add_argument("--bridge", default=os.path.join(ROOT, "bridge/build/sil_bridge"))
    ap.add_argument("--partition", default="tuba_sil_test", help="GZ_PARTITION for the private sim")
    ap.add_argument("--attach", action="store_true", help="gz sim + bridge are already running (default partition)")
    ap.add_argument("--no-mission", action="store_true", help="skip the deploy/dive step")
    ap.add_argument("--dive-depth", type=int, default=5)
    ap.add_argument("--keep", action="store_true", help="leave gz/bridge running at the end")
    ap.add_argument("--out", default="/tmp/tuba_sil_test", help="directory for logs")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    env = dict(os.environ, GZ_IP="127.0.0.1",
               GZ_SIM_RESOURCE_PATH=os.path.join(ROOT, "models") + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""))
    if not args.attach:
        env["GZ_PARTITION"] = args.partition
    procs = []

    def cleanup(*_):
        if args.keep:
            return
        for p in procs:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        time.sleep(1)
        for p in procs:
            if p.poll() is None:
                p.kill()

    try:
        if not args.attach:
            gz_log = open(os.path.join(args.out, "gz.log"), "w")
            gz = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "1", os.path.join(ROOT, "worlds/tuba_ocean.sdf")],
                                  env=env, stdout=gz_log, stderr=subprocess.STDOUT, cwd=ROOT)
            procs.append(gz)
            log(f"gz sim server started (pid {gz.pid}, partition {args.partition})")
            time.sleep(6)
            br_log = open(os.path.join(args.out, "bridge.log"), "w")
            br = subprocess.Popen([args.bridge, "--log", os.path.join(args.out, "bridge.csv")],
                                  env=env, stdin=subprocess.DEVNULL, stdout=br_log, stderr=subprocess.STDOUT, cwd=ROOT)
            procs.append(br)
            log(f"bridge started (pid {br.pid})")
            time.sleep(2)

        fw_logfile = open(os.path.join(args.out, "firmware.log"), "wb")
        fw = pexpect.spawn(args.firmware, ["--flash_in_ram"], env=env, timeout=30, logfile=fw_logfile)
        log(f"firmware started (pid {fw.pid})")

        fw.expect("press ENTER within")
        fw.send("\n")
        fw.expect(r"Select \[1-5\]")
        log("menu reached")

        # ---- 1. sensor scan --------------------------------------------------
        fw.send("2\n")
        fw.expect(r"Select \[1-9,a,x\]")
        fw.send("a\n")
        fw.expect("expected sensors present|expected sensor\\(s\\) missing", timeout=60)
        if "present" not in fw.after.decode():
            raise SystemExit("FAIL: sensor scan reports missing sensors (is the bridge running?)")
        log("PASS 1: all five sensors present on the emulated I2C buses")

        # ---- 2. actuators -----------------------------------------------------
        fw.expect(r"Select \[1-9,a,x\]")
        fw.send("2\n")                       # pump
        fw.expect("Enter seconds")
        fw.send("6\n")                       # 6 s out -> 0.6 L
        fw.expect(r"\[PUMP\] stopped", timeout=20)
        fw.send("q\n")
        fw.expect(r"Select \[1-9,a,x\]")
        fw.send("1\n")                       # pitch & roll
        fw.expect(r"Select \[1-3,x\]")
        fw.send("2\n")                       # pitch
        fw.expect("Enter seconds")
        fw.send("10\n")                      # 10 s fwd (limit switch sits at 12)
        fw.expect(r"\[PITCH\] timed stop", timeout=20)
        fw.send("q\n")
        fw.expect(r"Select \[1-3,x\]")
        fw.send("x\n")
        fw.expect(r"Select \[1-9,a,x\]")
        fw.send("3\n")                       # show positions
        fw.expect(r"\[POSITION\] roll=(-?\d+)s, pitch=(-?\d+)s, pump=(-?\d+)s")
        roll_s, pitch_s, pump_s = (int(x) for x in fw.match.groups())
        if (pitch_s, pump_s) != (10, 6):
            raise SystemExit(f"FAIL: positions pitch={pitch_s} pump={pump_s}, expected 10/6")
        log("PASS 2a: firmware dead-reckoning pitch=10 s, pump=6 s")

        time.sleep(25)                       # let it float up and settle
        fw.expect(r"Select \[1-9,a,x\]")
        fw.send("7\n")                       # compass
        fw.expect(r"Select \[1,2,x\]")
        fw.send("2\n")
        fw.expect(r"Heading=([\d.]+)°, Pitch=(-?[\d.]+)°, Roll=(-?[\d.]+)°", timeout=20)
        hdg, pitch_deg, roll_deg = (float(x) for x in fw.match.groups())
        fw.send("q\n")
        fw.expect(r"Select \[1,2,x\]")
        fw.send("x\n")
        log(f"compass: heading {hdg:.1f}, pitch {pitch_deg:.1f} (HMC6343 convention: - = nose-down), roll {roll_deg:.1f}")
        if not (-85 < pitch_deg < -40):
            raise SystemExit("FAIL: expected a steep nose-down float with the battery forward")
        log("PASS 2b: glider floats nose-down (battery forward, +1 N reserve)")

        fw.expect(r"Select \[1-9,a,x\]")
        fw.send("6\n")                       # GPS
        fw.expect(r"A (-?[\d.]+) (-?[\d.]+)", timeout=30)
        lat, lon = fw.match.groups()
        log(f"PASS 2c: GPS fix with the antenna out of the water: {lat}, {lon}")

        fw.expect(r"Select \[1-9,a,x\]")
        fw.send("5\n")                       # external pressure stream
        fw.expect(r"Depth=(-?[\d.]+) m", timeout=40)
        fw.send("q\n")
        fw.expect(r"Select \[1-9,a,x\]")
        fw.send("x\n")
        fw.expect(r"Select \[1-5\]")
        log("PASS 2d: MS5837 stream delivers depth")

        if args.no_mission:
            log("done (mission skipped)")
            return 0

        # ---- 3. mission --------------------------------------------------------
        params = {"1": args.dive_depth, "2": 5, "3": 5, "4": 4, "5": 6, "6": 6,
                  "7": 11, "9": 4, "a": 0, "h": 20, "k": 12, "l": 12}
        fw.send("1\n")
        fw.expect(r"Select \[1-9,a-m,s,r,x\]")
        for key, val in params.items():
            fw.send(f"{key}\n")
            fw.expect("Enter ")
            fw.send(f"{val}\n")
            fw.expect(r"Select \[1-9,a-m,s,r,x\]")
        fw.send("x\n")
        fw.expect(r"Select \[1-5\]")
        log(f"parameters set: dive {args.dive_depth} m, pump 6/4/6 s, pitch 11/4/0 s")

        fw.send("4\n")                       # deploy
        fw.expect(r"\[DEPLOY\] diving to", timeout=120)
        log("mission: diving")
        max_depth = 0.0
        t0 = time.time()
        while True:
            i = fw.expect([r"Depth=([\d.]+)m", r"target depth reached", r"DIVE ABORTED", r"dive timeout"], timeout=120)
            if i == 0:
                max_depth = max(max_depth, float(fw.match.group(1)))
            elif i == 1:
                break
            else:
                raise SystemExit(f"FAIL: dive ended early: {fw.after.decode()} (max depth {max_depth:.2f} m)")
            if time.time() - t0 > 600:
                raise SystemExit("FAIL: dive took longer than 10 min")
        log(f"PASS 3a: reached {max_depth:.2f} m -> climbing")
        fw.expect([r"moving to surface position", r"DIVE ABORTED"], timeout=600)
        if b"ABORTED" in fw.after:
            raise SystemExit("FAIL: climb aborted")
        log("PASS 3b: back near the surface")
        fw.expect("press ENTER to stop", timeout=120)
        fw.send("\n")
        fw.expect("mission complete", timeout=30)
        fw.expect(r"Select \[1-5\]", timeout=30)
        log("PASS 3c: mission stopped from the console, back in the menu")
        log("ALL PASSED")
        return 0
    except pexpect.TIMEOUT as e:
        print("FAIL: timeout waiting for the firmware. Last output:\n" + str(e)[:2000], file=sys.stderr)
        return 1
    finally:
        cleanup()


if __name__ == "__main__":
    sys.exit(main())
