/*
 * sil_protocol.h — Tuba software-in-the-loop (SIL) link packets
 *
 * Wire format between the firmware's virtual hardware layer (src/sil/) and
 * the Gazebo bridge (tuba-sim/bridge/sil_bridge.cpp). Both sides are
 * x86-64 Linux processes, so the structs are packed little-endian and used
 * as-is on the wire. THE COPY IN tuba-sim/bridge/sil_protocol.h MUST MATCH
 * THIS FILE — bump the magic numbers whenever the layout changes.
 *
 * Direction / cadence:
 *   firmware -> bridge  sil_act_pkt   20 Hz   what the H-bridges are doing
 *   bridge -> firmware  sil_sens_pkt  20 Hz   what the sensors would read
 *
 * The firmware never sends positions in physical units: it reports the
 * cumulative signed motor run time per actuator (seconds the pin pair was
 * driven, + = FWD / PORT / OUT), exactly what the real hardware "knows".
 * The bridge turns that into battery / bladder motion with the stroke
 * rates in its config, applies the end stops, and reports the limit
 * switches back. Dead-reckoning drift therefore behaves as on the glider.
 */
#ifndef SIL_PROTOCOL_H
#define SIL_PROTOCOL_H

#include <stdint.h>

#define SIL_MAGIC_ACT   0x54424131u   /* "TBA1" firmware -> bridge */
#define SIL_MAGIC_SENS  0x54425331u   /* "TBS1" bridge -> firmware */

#define SIL_FW_PORT     5600          /* firmware listens here (UDP) */
#define SIL_BRIDGE_PORT 5601          /* bridge listens here (UDP) */

/* sil_sens_pkt.flags */
#define SIL_F_GPS_FIX     (1u << 0)   /* antenna above the surface: RMC status A */
#define SIL_F_LIMIT_UP    (1u << 1)   /* pitch mass at the forward end stop */
#define SIL_F_LIMIT_DOWN  (1u << 2)   /* pitch mass at the aft end stop */
#define SIL_F_SIM_PAUSED  (1u << 3)   /* Gazebo is paused */
#define SIL_F_LEAK        (1u << 4)   /* leak probe wet (informational) */

struct __attribute__((packed)) sil_act_pkt {
    uint32_t magic;        /* SIL_MAGIC_ACT */
    uint32_t boot_id;      /* random per firmware boot; run_s restart from 0 */
    uint32_t seq;
    uint32_t uptime_ms;    /* firmware k_uptime */
    float    pitch_run_s;  /* cumulative signed drive time, + = FWD (nose-down) */
    float    roll_run_s;   /* + = PORT */
    float    pump_run_s;   /* + = OUT (more buoyant) */
    int8_t   pitch_dir;    /* current H-bridge state: -1 / 0 / +1 */
    int8_t   roll_dir;
    int8_t   pump_dir;
    uint8_t  flags;        /* reserved (HIL per-device source bits later) */
    int32_t  pitch_pos_s;  /* firmware's own dead-reckoned positions, display only */
    int32_t  roll_pos_s;
    int32_t  pump_pos_s;
    uint32_t crc;          /* sil_fnv1a() over all preceding bytes */
};

struct __attribute__((packed)) sil_sens_pkt {
    uint32_t magic;        /* SIL_MAGIC_SENS */
    uint32_t seq;
    uint32_t flags;        /* SIL_F_* */
    double   sim_time_s;   /* Gazebo sim time */
    int64_t  utc_unix_s;   /* host wall clock, for the GPS time fields */
    float    ext_pressure_pa;   /* MS5837 (water) absolute pressure */
    float    water_temp_c;      /* MS5837 + TSYS01 */
    float    int_pressure_pa;   /* BMP180 (hull) */
    float    int_temp_c;
    float    heading_deg;       /* HMC6343 convention: 0 = north, cw */
    float    pitch_deg;         /* HMC6343: + nose-up */
    float    roll_deg;          /* HMC6343: + starboard down */
    double   lat_deg;
    double   lon_deg;
    float    depth_m;           /* informational (console/debug) */
    float    leak_mv;           /* leak probe voltage at the ADC pin */
    uint32_t crc;
};

/* FNV-1a 32-bit over a byte range (tiny, no tables, same on both sides). */
static inline uint32_t sil_fnv1a(const void *buf, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    uint32_t h = 0x811C9DC5u;
    for (uint32_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 0x01000193u;
    }
    return h;
}

#endif /* SIL_PROTOCOL_H */
