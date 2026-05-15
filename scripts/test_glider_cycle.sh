#!/usr/bin/env bash
# test_glider_cycle.sh — execute one full sawtooth dive/ascend cycle and log results
#
# Prerequisites:
#   - gz sim running with tuba_ocean.sdf
#   - Glider spawned at z=-5 (default in world file)
#
# Usage: bash scripts/test_glider_cycle.sh [dive_depth] [num_cycles]
#   dive_depth: target depth in metres (default 30)
#   num_cycles: number of full dive+ascend cycles (default 2)

set -e

GLIDER="tuba_glider"
BUOY_TOPIC="/model/${GLIDER}/buoyancy_engine"
MASS_TOPIC="/model/${GLIDER}/joint/mass_shifter_joint/0/cmd_pos"
ALT_TOPIC="/model/${GLIDER}/altimeter"
POSE_TOPIC="/model/${GLIDER}/pose"

DIVE_DEPTH="${1:-30}"
NUM_CYCLES="${2:-2}"
SURFACE_DEPTH=3          # ascent endpoint depth (m)
POLL_INTERVAL=1.0        # seconds between depth polls
LOG_DIR="/tmp/tuba_cycle_$(date +%Y%m%d_%H%M%S)"

mkdir -p "${LOG_DIR}"
POSE_LOG="${LOG_DIR}/pose.txt"
echo "Logging pose to ${POSE_LOG}"

# Start background pose logger
timeout $((NUM_CYCLES * 600)) gz topic -e -t "${POSE_TOPIC}" > "${POSE_LOG}" &
POSE_PID=$!
trap "kill ${POSE_PID} 2>/dev/null; exit" INT TERM EXIT

echo "=== TUBA Glider Sawtooth Cycle Test ==="
echo "  Dive depth : ${DIVE_DEPTH} m"
echo "  Cycles     : ${NUM_CYCLES}"
echo ""

get_depth() {
    # Returns current depth (positive number = metres below surface)
    local alt
    alt=$(gz topic -e --duration=1 -t "${ALT_TOPIC}" 2>/dev/null \
          | grep -oP 'vertical_position: \K[-0-9.]+' | tail -1)
    # depth = -vertical_position
    echo "${alt:-0}" | awk '{printf "%.1f", -$1}'
}

for cycle in $(seq 1 "${NUM_CYCLES}"); do
    echo "--- Cycle ${cycle}/${NUM_CYCLES}: DESCEND ---"
    gz topic -t "${BUOY_TOPIC}" -m gz.msgs.Double -p "data: 0.001"
    gz topic -t "${MASS_TOPIC}" -m gz.msgs.Double -p "data: 0.12"

    while true; do
        depth=$(get_depth)
        echo "  depth = ${depth} m (target ${DIVE_DEPTH} m)"
        awk "BEGIN{exit (${depth} >= ${DIVE_DEPTH}) ? 0 : 1}" && break
        sleep "${POLL_INTERVAL}"
    done

    echo "--- Cycle ${cycle}/${NUM_CYCLES}: ASCEND ---"
    gz topic -t "${BUOY_TOPIC}" -m gz.msgs.Double -p "data: 0.003"
    gz topic -t "${MASS_TOPIC}" -m gz.msgs.Double -p "data: -0.12"

    while true; do
        depth=$(get_depth)
        echo "  depth = ${depth} m (target ${SURFACE_DEPTH} m)"
        awk "BEGIN{exit (${depth} <= ${SURFACE_DEPTH}) ? 0 : 1}" && break
        sleep "${POLL_INTERVAL}"
    done

    echo "--- Cycle ${cycle}: at surface (holding 5 s) ---"
    gz topic -t "${MASS_TOPIC}" -m gz.msgs.Double -p "data: 0.0"
    sleep 5
done

echo ""
echo "=== Cycle test complete. Log: ${POSE_LOG} ==="
echo "Run: python3 scripts/plot_trajectory.py --input ${POSE_LOG}"
