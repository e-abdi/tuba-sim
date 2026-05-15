/*
 * main.c — TUBA glider SIL firmware for Zephyr RTOS
 *
 * Boards:
 *   native_posix      — runs as native Linux process, simplest networking
 *   qemu_cortex_m3    — runs under QEMU with SLIRP networking (host at 10.0.2.2)
 *
 * Two threads:
 *   sensor_rx_thread  — binds UDP:5555, receives SensorPacket, updates shared state
 *   main (control)    — 10 Hz loop, runs state machine, sends ActuatorPacket to bridge
 *
 * State machine:
 *   IDLE    → wait for valid sensor, then descend
 *   DESCEND → negative buoyancy + mass forward → dive to DIVE_DEPTH_M
 *   ASCEND  → positive buoyancy + mass aft     → rise to SURFACE_DEPTH_M
 *   SURFACE → hold 30 s for simulated comms, then repeat
 */

#include <zephyr/kernel.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <math.h>

#include "glider_ctrl.h"

LOG_MODULE_REGISTER(glider, LOG_LEVEL_INF);

/* ── Shared sensor state ────────────────────────────────────────── */
K_MUTEX_DEFINE(sensor_mtx);
static SensorPacket g_sensor;
static bool         g_sensor_valid = false;

/* ── Sensor RX thread ───────────────────────────────────────────── */
#define RX_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(rx_stack, RX_STACK_SIZE);
static struct k_thread rx_thread_data;

static void sensor_rx_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        LOG_ERR("sensor socket failed: %d", sock);
        return;
    }

    struct sockaddr_in local = {
        .sin_family = AF_INET,
        .sin_port   = htons(SENSOR_PORT),
    };
    local.sin_addr.s_addr = INADDR_ANY;

    if (zsock_bind(sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        LOG_ERR("sensor bind failed");
        zsock_close(sock);
        return;
    }
    LOG_INF("Sensor RX listening on UDP:%d", SENSOR_PORT);

    SensorPacket pkt;
    while (1) {
        int n = zsock_recv(sock, &pkt, sizeof(pkt), 0);
        if (n != (int)sizeof(SensorPacket)) {
            LOG_WRN("Unexpected sensor packet size: %d", n);
            continue;
        }
        if (pkt.magic != SENSOR_MAGIC) {
            LOG_WRN("Bad sensor magic: 0x%08X", pkt.magic);
            continue;
        }
        uint32_t calc = glider_xor_checksum((uint8_t *)&pkt,
                                             sizeof(SensorPacket) - 4);
        if (calc != pkt.checksum) {
            LOG_WRN("Sensor checksum mismatch");
            continue;
        }
        k_mutex_lock(&sensor_mtx, K_FOREVER);
        g_sensor       = pkt;
        g_sensor_valid = true;
        k_mutex_unlock(&sensor_mtx);
    }
}

/* ── Actuator TX ────────────────────────────────────────────────── */
static int         act_sock = -1;
static struct sockaddr_in bridge_addr;

static void act_init(void)
{
    act_sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (act_sock < 0) {
        LOG_ERR("actuator socket failed: %d", act_sock);
        return;
    }
    memset(&bridge_addr, 0, sizeof(bridge_addr));
    bridge_addr.sin_family = AF_INET;
    bridge_addr.sin_port   = htons(ACTUATOR_PORT);
    zsock_inet_pton(AF_INET, BRIDGE_IP, &bridge_addr.sin_addr);
    LOG_INF("Actuator TX → %s:%d", BRIDGE_IP, ACTUATOR_PORT);
}

static void send_actuators(float volume, float mass_pos)
{
    ActuatorPacket pkt;
    pkt.magic         = ACTUATOR_MAGIC;
    pkt.bladder_vol_m3 = volume;
    pkt.mass_pos_m     = mass_pos;
    pkt.checksum = glider_xor_checksum((uint8_t *)&pkt,
                                        sizeof(ActuatorPacket) - 4);
    zsock_sendto(act_sock, &pkt, sizeof(pkt), 0,
                 (struct sockaddr *)&bridge_addr, sizeof(bridge_addr));
}

/* ── Main: control loop ─────────────────────────────────────────── */
int main(void)
{
    LOG_INF("TUBA Glider SIL starting");

    act_init();

    k_thread_create(&rx_thread_data, rx_stack, RX_STACK_SIZE,
                    sensor_rx_thread, NULL, NULL, NULL,
                    5, 0, K_NO_WAIT);

    GliderState state      = GLIDER_IDLE;
    float       cmd_volume = BLADDER_NEUTRAL;
    float       cmd_mass   = MASS_NEUTRAL_M;
    SensorPacket s;

    while (1) {
        k_msleep(100);  /* 10 Hz control loop */

        k_mutex_lock(&sensor_mtx, K_FOREVER);
        bool valid = g_sensor_valid;
        if (valid) {
            s = g_sensor;
        }
        k_mutex_unlock(&sensor_mtx);

        if (!valid) {
            /* No sensor data yet — keep neutral, wait */
            send_actuators(BLADDER_NEUTRAL, MASS_NEUTRAL_M);
            continue;
        }

        switch (state) {
        case GLIDER_IDLE:
            cmd_volume = BLADDER_NEUTRAL;
            cmd_mass   = MASS_NEUTRAL_M;
            if (s.depth_m > 1.0f) {
                LOG_INF("Sensor valid, depth=%.1f m — starting descent", (double)s.depth_m);
                state = GLIDER_DESCEND;
            }
            break;

        case GLIDER_DESCEND:
            cmd_volume = BLADDER_MIN_M3;
            cmd_mass   = MASS_FORE_M;
            LOG_DBG("DESCEND depth=%.1f m", (double)s.depth_m);
            if (s.depth_m >= DIVE_DEPTH_M) {
                LOG_INF("Reached dive depth %.1f m — ascending", (double)s.depth_m);
                state = GLIDER_ASCEND;
            }
            break;

        case GLIDER_ASCEND:
            cmd_volume = BLADDER_MAX_M3;
            cmd_mass   = MASS_AFT_M;
            LOG_DBG("ASCEND depth=%.1f m", (double)s.depth_m);
            if (s.depth_m <= SURFACE_DEPTH_M) {
                LOG_INF("Reached surface %.1f m — surfacing", (double)s.depth_m);
                state = GLIDER_SURFACE;
            }
            break;

        case GLIDER_SURFACE:
            cmd_volume = BLADDER_MAX_M3;
            cmd_mass   = MASS_NEUTRAL_M;
            LOG_INF("SURFACE hold (30 s simulated comms)");
            /* Simulated surface operations: hold 30 s then dive again */
            k_msleep(30000);
            LOG_INF("Restarting dive cycle");
            state = GLIDER_DESCEND;
            break;

        default:
            state = GLIDER_IDLE;
            break;
        }

        send_actuators(cmd_volume, cmd_mass);
    }

    return 0;
}
