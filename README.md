# TUBA Glider Simulation

Gazebo gz-sim 10 simulation of the [TUBA open-source underwater glider](https://hackaday.io/project/196850-tuba-the-open-source-glider), including Zephyr RTOS SIL (software-in-the-loop) integration.

## Prerequisites

| Tool | Version |
|------|---------|
| gz-sim | 10.2.0 |
| Python | 3.10+ |
| gz Python bindings | system install (`python3-gz-transport*`) |
| Zephyr SDK | 0.16+ (for Phase 4 only) |
| `west` | 1.2+ (for Phase 4 only) |

```bash
# Install Python plotting deps
pip install -r bridge/requirements.txt
```

## Project Layout

```
worlds/            Ocean world SDF
models/tuba_glider/ Glider model (SDF + config)
config/            Physics tuning parameters
scripts/           Launch and test scripts
bridge/            Gazebo ↔ Zephyr UDP bridge
zephyr/            Zephyr RTOS firmware skeleton
docs/              FSD and architecture docs
```

## Phase 1 & 2 — Run the Simulation

```bash
# Launch Gazebo with the ocean world + glider
bash scripts/run_sim.sh

# Or with debug verbosity:
bash scripts/run_sim.sh 4
```

The glider spawns 5 m below the surface in neutral buoyancy.

## Phase 2 — Manual Control

```bash
# Rise (max positive buoyancy)
gz topic -t /model/tuba_glider/buoyancy_engine -m gz.msgs.Double -p "data: 0.003"

# Sink (max negative buoyancy)
gz topic -t /model/tuba_glider/buoyancy_engine -m gz.msgs.Double -p "data: 0.001"

# Neutral
gz topic -t /model/tuba_glider/buoyancy_engine -m gz.msgs.Double -p "data: 0.002"

# Pitch nose-down (mass forward → dive attitude)
gz topic -t /model/tuba_glider/joint/mass_shifter_joint/0/cmd_pos -m gz.msgs.Double -p "data: 0.12"

# Pitch nose-up (mass aft → ascend attitude)
gz topic -t /model/tuba_glider/joint/mass_shifter_joint/0/cmd_pos -m gz.msgs.Double -p "data: -0.12"

# Monitor depth
gz topic -e -t /model/tuba_glider/altimeter

# Monitor IMU
gz topic -e -t /model/tuba_glider/imu
```

## Phase 3 — Buoyancy Smoke Test

```bash
bash scripts/test_buoyancy.sh
```

## Phase 3 — Full Glide Cycle Test

```bash
# Two cycles, dive to 30 m (default)
bash scripts/test_glider_cycle.sh

# Custom: 3 cycles, dive to 20 m
bash scripts/test_glider_cycle.sh 20 3
```

Plot the logged trajectory:

```bash
python3 scripts/plot_trajectory.py --input /tmp/pose_log.txt
```

## Phase 4 — Zephyr SIL

### native_posix (recommended for dev)

```bash
cd zephyr
west build -b native_posix
./build/zephyr/zephyr.exe &

# In another terminal — start Gazebo first, then bridge
python3 ../bridge/gz_zephyr_bridge.py --verbose
```

### qemu_cortex_m3

```bash
cd zephyr
west build -b qemu_cortex_m3

# Launch QEMU with SLIRP networking (host appears at 10.0.2.2 inside QEMU)
qemu-system-arm -M mps2-an385 -cpu cortex-m3 -nographic \
  -kernel build/zephyr/zephyr.elf \
  -netdev user,id=eth0,hostfwd=udp::5555-:5555,hostfwd=udp::5556-:5556 \
  -net nic,model=lan9118,netdev=eth0 &

python3 ../bridge/gz_zephyr_bridge.py
```

## Physics Tuning

Edit `config/physics_params.yaml` and manually copy the values into `models/tuba_glider/model.sdf` plugin sections. Restart gz sim after each change.

Key coefficients to tune:
- `xUabsU` — reduce for less axial drag (faster forward speed)
- `cla` (LiftDrag) — increase for more wing lift at low AoA
- `neutral_volume` — adjust so glider is exactly neutral at desired depth

## Integrating the Real Glider Shell

Replace the cylinder placeholder in `models/tuba_glider/model.sdf` with your actual mesh:

```xml
<visual name="hull_visual">
  <geometry>
    <mesh><uri>model://tuba_glider/meshes/hull.dae</uri></mesh>
  </geometry>
</visual>
<collision name="hull_collision">
  <geometry>
    <mesh><uri>model://tuba_glider/meshes/hull_collision.dae</uri></mesh>
  </geometry>
</collision>
```

Place mesh files in `models/tuba_glider/meshes/`. Update the inertia and mass values to match the real glider.

## Known Issues / Open Items

1. Fuel model URLs (`Coast Water`, `OceanFloorShipwreck`) must be verified with `gz fuel download` before first run.
2. The hull geometry uses a cylinder placeholder — replace with real CAD mesh for accurate drag and buoyancy volume.
3. Hydrodynamics coefficients are analytical estimates — expect 1–2 tuning iterations.
4. `qemu_cortex_m3` board requires Zephyr Ethernet driver (`CONFIG_ETH_SMSC911X`) which must be available in your Zephyr SDK version.
