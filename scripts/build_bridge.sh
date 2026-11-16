#!/usr/bin/env bash
# build_bridge.sh — compile bridge/sil_bridge (needs libgz-transport15-dev, libgz-msgs12-dev)
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "${ROOT}/bridge" -B "${ROOT}/bridge/build" -DCMAKE_BUILD_TYPE=Release > /dev/null
cmake --build "${ROOT}/bridge/build" -- -j"$(nproc)"
echo "Built: ${ROOT}/bridge/build/sil_bridge"
