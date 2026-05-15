#!/usr/bin/env bash
# run_sim.sh — launch the TUBA glider ocean world in gz-sim 10

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export GZ_SIM_RESOURCE_PATH="${PROJECT_ROOT}/models:${GZ_SIM_RESOURCE_PATH}"

WORLD="${PROJECT_ROOT}/worlds/tuba_ocean.sdf"
VERBOSITY="${1:-3}"   # default verbosity 3; pass 4 for debug

echo "[run_sim] GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
echo "[run_sim] Launching: gz sim -v ${VERBOSITY} ${WORLD}"

exec gz sim -v "${VERBOSITY}" "${WORLD}"
