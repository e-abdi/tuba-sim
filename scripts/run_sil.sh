#!/usr/bin/env bash
# run_sil.sh — start the Gazebo <-> firmware bridge (software/hardware-in-the-loop)
#
# Terminal 1:  bash scripts/run_sim.sh            Gazebo (press Play)
# Terminal 2:  bash scripts/run_sil.sh            this bridge
# Terminal 3:  ~/zephyrproject/tuba/sil_run.sh    the firmware's console
#
# Extra arguments go to the bridge, e.g.
#   bash scripts/run_sil.sh --log /tmp/run.csv
#   bash scripts/run_sil.sh --set fw_host=192.168.4.1     # real ESP32 over WiFi
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export GZ_IP="${GZ_IP:-127.0.0.1}"
if [ ! -x "${ROOT}/bridge/build/sil_bridge" ]; then
    echo "[run_sil] bridge not built yet — running scripts/build_bridge.sh"
    bash "${ROOT}/scripts/build_bridge.sh"
fi
cd "${ROOT}"
exec bridge/build/sil_bridge --config config/sil.cfg "$@"
