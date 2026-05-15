#!/usr/bin/env python3
"""
gz_zephyr_bridge.py — Gazebo gz-sim 10 ↔ Zephyr RTOS SIL bridge

Runs as a standalone process (no ROS 2 required).
Connects gz-transport sensor topics to a Zephyr firmware running in QEMU
(or native_posix) via UDP datagrams.

Data flow:
  Gazebo IMU + Altimeter → SensorPacket (UDP) → Zephyr port 5555
  Zephyr ActuatorPacket  (UDP port 5556)       → Gazebo buoyancy + mass topics

Usage:
  python3 bridge/gz_zephyr_bridge.py [--zephyr-host 127.0.0.1]
                                      [--sensor-port 5555]
                                      [--actuator-port 5556]
                                      [--verbose]

Dependencies (system install via gz-sim 10 apt packages):
  gz.transport   (Python bindings — package: python3-gz-transport<N>)
  gz.msgs        (protobuf bindings — package: python3-gz-msgs<N>)
"""

import argparse
import math
import signal
import socket
import struct
import sys
import threading
import time

from protocol import (
    pack_sensor, unpack_actuator,
    SENSOR_PORT, ACTUATOR_PORT,
)

try:
    from gz.transport13 import Node
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.altimeter_pb2 import Altimeter
    from gz.msgs10.double_pb2 import Double
except ImportError:
    # Version numbers in gz Python bindings vary; try generic import
    try:
        from gz.transport import Node          # type: ignore
        from gz.msgs.imu_pb2 import IMU        # type: ignore
        from gz.msgs.altimeter_pb2 import Altimeter  # type: ignore
        from gz.msgs.double_pb2 import Double  # type: ignore
    except ImportError as e:
        sys.exit(
            f"Could not import gz Python bindings: {e}\n"
            "Install with: sudo apt install python3-gz-transport<N> python3-gz-msgs<N>\n"
            "where <N> matches your gz-sim version (e.g. 13 for gz-sim 10)."
        )

GLIDER_NS  = "tuba_glider"
IMU_TOPIC  = f"/model/{GLIDER_NS}/imu"
ALT_TOPIC  = f"/model/{GLIDER_NS}/altimeter"
BUOY_TOPIC = f"/model/{GLIDER_NS}/buoyancy_engine"
MASS_TOPIC = f"/model/{GLIDER_NS}/joint/mass_shifter_joint/0/cmd_pos"

SENSOR_RATE_HZ = 10.0

# Shared sensor state — written by gz callbacks, read by UDP TX thread
_lock   = threading.Lock()
_sensor = {
    "sim_time_ns": 0,
    "depth_m":   0.0,
    "pitch":     0.0,
    "roll":      0.0,
    "yaw":       0.0,
    "acc_x":     0.0, "acc_y": 0.0, "acc_z": -9.81,
    "gyro_x":    0.0, "gyro_y": 0.0, "gyro_z": 0.0,
}
_running = True
_verbose = False


def _q_to_euler(qx, qy, qz, qw):
    """Quaternion → roll, pitch, yaw (radians)."""
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)

    return roll, pitch, yaw


def _on_imu(msg: IMU):
    ts   = msg.header.stamp
    q    = msg.orientation
    a    = msg.linear_acceleration
    g    = msg.angular_velocity
    roll, pitch, yaw = _q_to_euler(q.x, q.y, q.z, q.w)
    with _lock:
        _sensor.update({
            "sim_time_ns": ts.sec * 1_000_000_000 + ts.nsec,
            "pitch": pitch, "roll": roll, "yaw": yaw,
            "acc_x": a.x, "acc_y": a.y, "acc_z": a.z,
            "gyro_x": g.x, "gyro_y": g.y, "gyro_z": g.z,
        })


def _on_altimeter(msg: Altimeter):
    # depth_m = -vertical_position  (z is negative underwater)
    with _lock:
        _sensor["depth_m"] = -msg.vertical_position


def _sensor_tx(sock: socket.socket, addr: tuple):
    period = 1.0 / SENSOR_RATE_HZ
    while _running:
        t0 = time.monotonic()
        with _lock:
            s = dict(_sensor)
        pkt = pack_sensor(
            s["sim_time_ns"], s["depth_m"],
            s["pitch"], s["roll"], s["yaw"],
            s["acc_x"], s["acc_y"], s["acc_z"],
            s["gyro_x"], s["gyro_y"], s["gyro_z"],
        )
        try:
            sock.sendto(pkt, addr)
            if _verbose:
                print(f"[bridge tx] depth={s['depth_m']:.2f} m  "
                      f"pitch={math.degrees(s['pitch']):.1f}°")
        except OSError as e:
            print(f"[bridge] sendto error: {e}")
        elapsed = time.monotonic() - t0
        time.sleep(max(0.0, period - elapsed))


def _actuator_rx(sock: socket.socket, pub_buoy, pub_mass):
    sock.settimeout(1.0)
    while _running:
        try:
            data, addr = sock.recvfrom(64)
        except socket.timeout:
            continue
        result = unpack_actuator(data)
        if result is None:
            print(f"[bridge] Bad actuator packet from {addr} ({len(data)} bytes)")
            continue
        bladder, mass_pos = result
        if _verbose:
            print(f"[bridge rx] bladder={bladder:.4f} m³  mass={mass_pos:.3f} m")
        b_msg = Double()
        b_msg.data = bladder
        pub_buoy.publish(b_msg)

        m_msg = Double()
        m_msg.data = mass_pos
        pub_mass.publish(m_msg)


def main():
    global _running, _verbose

    parser = argparse.ArgumentParser(description="Gazebo ↔ Zephyr SIL bridge")
    parser.add_argument("--zephyr-host",   default="127.0.0.1",
                        help="IP address of Zephyr QEMU (default: 127.0.0.1)")
    parser.add_argument("--sensor-port",   type=int, default=SENSOR_PORT,
                        help=f"UDP port for sensor data → Zephyr (default: {SENSOR_PORT})")
    parser.add_argument("--actuator-port", type=int, default=ACTUATOR_PORT,
                        help=f"UDP port for actuator data ← Zephyr (default: {ACTUATOR_PORT})")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Print each packet sent/received")
    args = parser.parse_args()
    _verbose = args.verbose

    # gz-transport node
    node = Node()
    node.subscribe(IMU,       IMU_TOPIC,  _on_imu)
    node.subscribe(Altimeter, ALT_TOPIC,  _on_altimeter)
    pub_buoy = node.advertise(BUOY_TOPIC, Double)
    pub_mass = node.advertise(MASS_TOPIC, Double)

    print(f"[bridge] Subscribed: {IMU_TOPIC}")
    print(f"[bridge] Subscribed: {ALT_TOPIC}")
    print(f"[bridge] Publishing: {BUOY_TOPIC}")
    print(f"[bridge] Publishing: {MASS_TOPIC}")

    zephyr_addr      = (args.zephyr_host, args.sensor_port)
    local_act_addr   = ("", args.actuator_port)

    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.bind(local_act_addr)

    print(f"[bridge] Sending sensors to {zephyr_addr}")
    print(f"[bridge] Listening for actuators on port {args.actuator_port}")

    def _shutdown(sig, frame):
        global _running
        _running = False

    signal.signal(signal.SIGINT,  _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    t_tx = threading.Thread(target=_sensor_tx,
                             args=(tx_sock, zephyr_addr), daemon=True)
    t_rx = threading.Thread(target=_actuator_rx,
                             args=(rx_sock, pub_buoy, pub_mass), daemon=True)
    t_tx.start()
    t_rx.start()

    print("[bridge] Running. Ctrl-C to stop.")
    while _running:
        time.sleep(0.1)

    tx_sock.close()
    rx_sock.close()
    print("[bridge] Stopped.")


if __name__ == "__main__":
    main()
