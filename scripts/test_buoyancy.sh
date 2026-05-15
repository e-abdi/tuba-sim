#!/usr/bin/env bash
# test_buoyancy.sh — verify rise/sink/neutral behaviour via gz topic commands
# Run while gz sim is already open with tuba_ocean.sdf

set -e

GLIDER="tuba_glider"
BUOY_TOPIC="/model/${GLIDER}/buoyancy_engine"
MASS_TOPIC="/model/${GLIDER}/joint/mass_shifter_joint/0/cmd_pos"
ALT_TOPIC="/model/${GLIDER}/altimeter"

echo "=== TUBA Glider Buoyancy Smoke Test ==="
echo "Make sure gz sim is running with tuba_ocean.sdf"
echo ""

echo "--- Step 1: Set NEUTRAL buoyancy (volume=0.002 m³) ---"
gz topic -t "${BUOY_TOPIC}" -m gz.msgs.Double -p "data: 0.002"
gz topic -t "${MASS_TOPIC}" -m gz.msgs.Double -p "data: 0.0"
echo "Monitoring altimeter for 5 s (expect stable ~-5 m if spawned at z=-5):"
timeout 5 gz topic -e -t "${ALT_TOPIC}" | grep -oP 'vertical_position: \K[0-9.\-]+' || true
echo ""

echo "--- Step 2: MAX positive buoyancy (volume=0.003 m³) → should rise ---"
gz topic -t "${BUOY_TOPIC}" -m gz.msgs.Double -p "data: 0.003"
echo "Monitoring altimeter for 8 s (expect vertical_position increasing):"
timeout 8 gz topic -e -t "${ALT_TOPIC}" | grep -oP 'vertical_position: \K[0-9.\-]+' || true
echo ""

echo "--- Step 3: MAX negative buoyancy (volume=0.001 m³) → should sink ---"
gz topic -t "${BUOY_TOPIC}" -m gz.msgs.Double -p "data: 0.001"
echo "Monitoring altimeter for 8 s (expect vertical_position decreasing):"
timeout 8 gz topic -e -t "${ALT_TOPIC}" | grep -oP 'vertical_position: \K[0-9.\-]+' || true
echo ""

echo "--- Step 4: Return to neutral ---"
gz topic -t "${BUOY_TOPIC}" -m gz.msgs.Double -p "data: 0.002"
echo "Done. Check gz sim GUI to visually confirm glider position."
