# TUBA Glider Simulation

Gazebo simulation of the [TUBA open-source underwater glider](https://hackaday.io/project/196850-tuba-the-open-source-glider), including Zephyr RTOS SIL (software-in-the-loop) integration.

![TUBA Simulation](pics/tuba-sim.png)

## Models & Assets

- **Glider Hull**: TUBA shell converted to SDF using [FusionSDF](https://github.com/andreasBihlmaier/FusionSDF)
- **Water Surface**: [Coast Water](https://app.gazebosim.org/OpenRobotics/fuel/models/Coast%20Water) model from Gazebo Fuel
- **Seafloor**: [Coral Reef Outcrop](https://sketchfab.com/3d-models/coral-reef-outcrop-8bc72c9e4575470ea30d1313e48b730c) model from Sketchfab (run `unzip models/seafloor.zip "coral/seafloor/*" -d /tmp && mv /tmp/coral/seafloor models/` before launching)

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

The glider spawns floating at the surface, ~70° nose-down with the antenna out of the water.

## Dive Mission

With the sim running, in a second terminal:

```bash
python3 scripts/glider_mission.py                 # repeat dive cycles forever
python3 scripts/glider_mission.py --cycles 1 --log /tmp/mission.csv
```

Each cycle: float at the surface (~70° nose-down, antenna out) → dive steeply (~75°) to 2 m
→ glide down at 30° nose-down to 10 m → turn buoyant and climb at 30° nose-up to 1 m
→ pitch over to ~75° nose-down and rise tail-first until the antenna is out again.
Depths, pitch, volumes and gains are all command-line options (`--help`).

## Phase 2 — Manual Control

The world spawns `my_glider_v2`. Bladder volume range is 0–1.2 L (0.0005 = neutral).
Don't run manual commands while `glider_mission.py` is running; it overrides them.

`run_sim.sh` pins gz-transport to loopback (`GZ_IP=127.0.0.1`) so other terminals reliably
find the sim on machines with several network interfaces. Run `export GZ_IP=127.0.0.1` in any
terminal where you use `gz topic` / `gz service` (`glider_mission.py` sets it itself).

```bash
# Surface float (+1 N reserve; with battery fully forward floats ~70° nose-down)
gz topic -t /model/my_glider_v2/buoyancy_engine -m gz.msgs.Double -p "data: 0.0006"

# Glide down (-1.5 N) / glide up (+1.5 N)
gz topic -t /model/my_glider_v2/buoyancy_engine -m gz.msgs.Double -p "data: 0.00035"
gz topic -t /model/my_glider_v2/buoyancy_engine -m gz.msgs.Double -p "data: 0.00065"

# Battery (mass shifter), range -0.05..0.14 m, positive = toward the nose.
# Submerged pitch ~= atan((battery + 0.0195) / 0.0428), positive = nose-down:
#   -0.05 → ~35° nose-up, -0.0195 → level, 0.005 → 30° down, 0.14 → 75° down
gz topic -t /my_glider_v2/battery_joint_pos -m gz.msgs.Double -p "data: 0.005"

# Monitor bladder volume / joint positions
gz topic -e -t /model/my_glider_v2/buoyancy_engine/current_volume
gz topic -e -t /world/tuba_ocean/model/my_glider_v2/joint_state
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
