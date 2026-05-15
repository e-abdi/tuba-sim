"""
protocol.py — TUBA SIL bridge binary packet definitions.

SensorPacket  (bridge → Zephyr, UDP port 5555, 56 bytes)
ActuatorPacket (Zephyr → bridge, UDP port 5556, 16 bytes)

All fields are little-endian.
Checksum is XOR of all preceding bytes in the packet.
"""

import struct

SENSOR_MAGIC   = 0xDEADBEEF
ACTUATOR_MAGIC = 0xBEEFCAFE

SENSOR_PORT   = 5555
ACTUATOR_PORT = 5556

# SensorPacket layout (56 bytes):
#   magic(u32) sim_time_ns(u64) depth_m(f32) pitch(f32) roll(f32) yaw(f32)
#   acc_x(f32) acc_y(f32) acc_z(f32) gyro_x(f32) gyro_y(f32) gyro_z(f32)
#   checksum(u32)
_SENSOR_FMT  = "<IQffffffffffff I"
SENSOR_SIZE  = struct.calcsize(_SENSOR_FMT)   # 56 bytes

# ActuatorPacket layout (16 bytes):
#   magic(u32) bladder_volume_m3(f32) mass_position_m(f32) checksum(u32)
_ACTUATOR_FMT = "<IffI"
ACTUATOR_SIZE  = struct.calcsize(_ACTUATOR_FMT)  # 16 bytes

# Safe operating ranges (bridge clamps actuator commands to these)
BLADDER_MIN_M3 = 0.001
BLADDER_MAX_M3 = 0.003
MASS_MIN_M     = -0.15
MASS_MAX_M     =  0.15


def _xor_checksum(data: bytes) -> int:
    chk = 0
    for b in data:
        chk ^= b
    return chk & 0xFFFFFFFF


def pack_sensor(sim_time_ns: int, depth_m: float,
                pitch: float, roll: float, yaw: float,
                acc_x: float, acc_y: float, acc_z: float,
                gyro_x: float, gyro_y: float, gyro_z: float) -> bytes:
    """Encode a SensorPacket. Returns 56-byte bytes object."""
    body = struct.pack(
        "<IQffffffffffff",
        SENSOR_MAGIC, sim_time_ns,
        depth_m, pitch, roll, yaw,
        acc_x, acc_y, acc_z,
        gyro_x, gyro_y, gyro_z,
    )
    checksum = _xor_checksum(body)
    return body + struct.pack("<I", checksum)


def unpack_actuator(data: bytes):
    """Decode an ActuatorPacket. Returns (bladder_m3, mass_pos_m) or None."""
    if len(data) != ACTUATOR_SIZE:
        return None
    magic, bladder, mass_pos, checksum = struct.unpack(_ACTUATOR_FMT, data)
    if magic != ACTUATOR_MAGIC:
        return None
    calc = _xor_checksum(data[:ACTUATOR_SIZE - 4])
    if calc != checksum:
        return None
    bladder  = max(BLADDER_MIN_M3, min(BLADDER_MAX_M3, bladder))
    mass_pos = max(MASS_MIN_M,     min(MASS_MAX_M,     mass_pos))
    return bladder, mass_pos
