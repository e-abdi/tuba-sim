# TUBA Glider Simulation

Gazebo gz-sim 10 simulation of the [TUBA](https://github.com/e-abdi) open-source underwater glider,
with Zephyr RTOS SIL (software-in-the-loop) integration.

The glider uses a buoyancy-engine piston for depth control and a sliding battery mass for pitch
control — the same actuators as the physical vehicle being built alongside this sim.

## Status

| Phase | Description | State |
|-------|-------------|-------|
| 1 | Ocean world (graded buoyancy, seafloor, lighting) | Done |
| 2 | Glider model (hull, wings, mass shifter, sensors) | Done |
| 3 | Physics tuning (hydrodynamics coefficients) | Needs real sim run |
| 4 | Zephyr SIL bridge (QEMU ↔ Gazebo UDP) | Scaffolded — needs Zephyr SDK |

## Prerequisites

| Tool | Version |
|------|---------|
| gz-sim | 10.2.0 |
| Python | 3.10+ |
| gz Python bindings | system install (`python3-gz-transport*`) |
| Zephyr SDK | 0.16+ (Phase 4 only) |
| `west` | 1.2+ (Phase 4 only) |

```bash
pip install -r bridge/requirements.txt
```

## Project Layout

```
worlds/             Ocean world SDF
models/tuba_glider/ Glider model (SDF + config)
config/             Physics tuning parameters
scripts/            Launch and test scripts
bridge/             Gazebo ↔ Zephyr UDP bridge
zephyr/             Zephyr RTOS firmware skeleton
docs/               FSD and architecture docs
```

## Running the Simulation

```bash
# Launch Gazebo with the ocean world + glider
bash scripts/run_sim.sh

# Debug verbosity
bash scripts/run_sim.sh 4
```

The glider spawns 3 m below the surface (z = 17) in neutral buoyancy.

## Manual Control

```bash
# Rise (max buoyancy — 1 L bladder)
gz topic -t /model/my_glider_v2/buoyancy_engine -m gz.msgs.Double -p "data: 0.001"

# Sink (empty bladder)
gz topic -t /model/my_glider_v2/buoyancy_engine -m gz.msgs.Double -p "data: 0.0"

# Neutral
gz topic -t /model/my_glider_v2/buoyancy_engine -m gz.msgs.Double -p "data: 0.0005"

# Pitch nose-down (VBD piston forward → dive attitude)
gz topic -t /my_glider_v2/vbd_joint_pos -m gz.msgs.Double -p "data: 0.04"

# Pitch nose-up (VBD piston aft → ascend attitude)
gz topic -t /my_glider_v2/vbd_joint_pos -m gz.msgs.Double -p "data: 0.01"

# Shift battery mass forward (nose down)
gz topic -t /my_glider_v2/battery_joint_pos -m gz.msgs.Double -p "data: 0.05"

# Shift battery mass aft (nose up)
gz topic -t /my_glider_v2/battery_joint_pos -m gz.msgs.Double -p "data: -0.05"

# Monitor depth
gz topic -e -t /model/my_glider_v2/altimeter

# Monitor IMU
gz topic -e -t /model/my_glider_v2/imu
```

## Tests

```bash
# Buoyancy smoke test
bash scripts/test_buoyancy.sh

# Full glide cycle (two cycles, dive to 30 m by default)
bash scripts/test_glider_cycle.sh

# Custom: 3 cycles, dive to 20 m
bash scripts/test_glider_cycle.sh 20 3

# Plot logged trajectory
python3 scripts/plot_trajectory.py --input /tmp/pose_log.txt
```

## Phase 4 — Zephyr SIL

### native_posix (recommended for dev)

```bash
cd zephyr
west build -b native_posix
./build/zephyr/zephyr.exe &

# In a second terminal — start Gazebo first, then the bridge
python3 ../bridge/gz_zephyr_bridge.py --verbose
```

### qemu_cortex_m3

```bash
cd zephyr
west build -b qemu_cortex_m3

qemu-system-arm -M mps2-an385 -cpu cortex-m3 -nographic \
  -kernel build/zephyr/zephyr.elf \
  -netdev user,id=eth0,hostfwd=udp::5555-:5555,hostfwd=udp::5556-:5556 \
  -net nic,model=lan9118,netdev=eth0 &

python3 ../bridge/gz_zephyr_bridge.py
```

## Physics Tuning

Edit `config/physics_params.yaml` and copy the values into `models/my_glider_v2/model.sdf`.
Restart gz-sim after each change.

Key coefficients:
- `xUabsU` — reduce for less axial drag (faster forward speed)
- `cla` (LiftDrag) — increase for more wing lift at low AoA
- `neutral_volume` — adjust so glider is exactly neutral at desired depth

## Known Issues

1. Fuel model URL (`Coast Water`) must be verified with `gz fuel download` before first run.
2. Hydrodynamics coefficients are analytical estimates — expect 1–2 tuning iterations after first sim run.
3. `qemu_cortex_m3` board requires `CONFIG_ETH_SMSC911X` which must be available in your Zephyr SDK version.
