#!/usr/bin/env bash
# run_sim.sh — launch the TUBA glider ocean world in gz-sim 10

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

export GZ_SIM_RESOURCE_PATH="${PROJECT_ROOT}/models:${GZ_SIM_RESOURCE_PATH}"
# Keep gz-transport on loopback. With several interfaces (Wi-Fi, VPN, docker)
# discovery can pick different ones per process, and scripts in another
# terminal then never see the sim. Other terminals need the same GZ_IP.
export GZ_IP="${GZ_IP:-127.0.0.1}"

WORLD="${PROJECT_ROOT}/worlds/tuba_ocean.sdf"
VERBOSITY="${1:-3}"   # default verbosity 3; pass 4 for debug

echo "[run_sim] GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
echo "[run_sim] GZ_IP=${GZ_IP}"
echo "[run_sim] Launching: gz sim -v ${VERBOSITY} ${WORLD}"

if gz help 2>/dev/null | grep -q "^  sim:"; then
    exec gz sim -v "${VERBOSITY}" "${WORLD}"
fi
# No `gz sim` command: the gz-sim10 CLI/GUI package is missing (this happens
# after an Ubuntu release upgrade disables the packages.osrfoundation.org
# source). Fall back to the headless server if it is still installed.
SERVER=/usr/libexec/gz/sim10/gz-sim-server
if [ -x "${SERVER}" ]; then
    echo "[run_sim] 'gz sim' not available — running the HEADLESS server (${SERVER})."
    echo "[run_sim] Re-enable the Gazebo apt source and 'sudo apt install gz-sim10' for the GUI."
    exec "${SERVER}" -s -r -v "${VERBOSITY}" "${WORLD}"
fi
echo "[run_sim] Gazebo is not installed (no 'gz sim', no ${SERVER})" >&2
exit 1
