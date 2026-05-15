/*
 * glider_ctrl.h — TUBA glider SIL: shared types and protocol structs
 *
 * Binary protocol (little-endian, matches bridge/protocol.py):
 *
 * SensorPacket  — bridge → Zephyr, UDP port 5555, 56 bytes
 * ActuatorPacket — Zephyr → bridge, UDP port 5556, 16 bytes
 *
 * Checksum: XOR of all bytes preceding the checksum field.
 */

#ifndef GLIDER_CTRL_H
#define GLIDER_CTRL_H

#include <stdint.h>
#include <stdbool.h>

/* ── Protocol constants ─────────────────────────────────────────── */
#define SENSOR_MAGIC    0xDEADBEEFU
#define ACTUATOR_MAGIC  0xBEEFCAFEU

#define SENSOR_PORT     5555
#define ACTUATOR_PORT   5556

/* For QEMU SLIRP: the host machine appears at this IP inside QEMU.
 * For native_posix: use "127.0.0.1" instead. */
#define BRIDGE_IP       "10.0.2.2"

/* ── Packet structures ──────────────────────────────────────────── */

typedef struct __attribute__((packed)) {
    uint32_t magic;          /* SENSOR_MAGIC */
    uint64_t sim_time_ns;    /* simulation time (nanoseconds) */
    float    depth_m;        /* depth below surface (m, positive = deeper) */
    float    pitch_rad;      /* body pitch (+up = nose-up) */
    float    roll_rad;       /* body roll */
    float    yaw_rad;        /* body yaw (heading) */
    float    acc_x;          /* linear acceleration X (m/s², body frame) */
    float    acc_y;
    float    acc_z;
    float    gyro_x;         /* angular velocity X (rad/s, body frame) */
    float    gyro_y;
    float    gyro_z;
    uint32_t checksum;       /* XOR of bytes [0..51] */
} SensorPacket;              /* 56 bytes total */

typedef struct __attribute__((packed)) {
    uint32_t magic;          /* ACTUATOR_MAGIC */
    float    bladder_vol_m3; /* buoyancy engine volume setpoint (m³) */
    float    mass_pos_m;     /* mass shifter position setpoint (m) */
    uint32_t checksum;       /* XOR of bytes [0..11] */
} ActuatorPacket;            /* 16 bytes total */

/* ── Glider state machine ───────────────────────────────────────── */
typedef enum {
    GLIDER_IDLE    = 0,
    GLIDER_DESCEND = 1,
    GLIDER_ASCEND  = 2,
    GLIDER_SURFACE = 3,
} GliderState;

/* ── Physical limits ────────────────────────────────────────────── */
#define BLADDER_MIN_M3  0.001f
#define BLADDER_NEUTRAL 0.002f
#define BLADDER_MAX_M3  0.003f

#define MASS_FORE_M     0.12f    /* nose-down: max forward position */
#define MASS_AFT_M     -0.12f   /* nose-up:   max aft position */
#define MASS_NEUTRAL_M  0.0f

#define DIVE_DEPTH_M    30.0f   /* target dive depth (metres below surface) */
#define SURFACE_DEPTH_M  3.0f   /* ascent endpoint depth */

/* ── Checksum helper (inline, usable in both TX and RX) ─────────── */
static inline uint32_t glider_xor_checksum(const uint8_t *data, uint32_t len)
{
    uint32_t chk = 0;
    for (uint32_t i = 0; i < len; i++) {
        chk ^= (uint32_t)data[i];
    }
    return chk;
}

#endif /* GLIDER_CTRL_H */
