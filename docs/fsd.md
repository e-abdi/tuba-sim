# TUBA Glider Simulation — Functional Specification Document

Version 1.0 | Gazebo gz-sim 10.2.0 | Standalone (no ROS 2)

---

## 1. Purpose

This document specifies the complete simulation environment for the TUBA open-source underwater glider. It covers:
- Ocean world construction (Gazebo)
- Glider physics model (buoyancy, hydrodynamics, lift)
- Control interface (gz-transport topics)
- Zephyr RTOS software-in-the-loop (SIL) integration

---

## 2. System Overview

```
┌──────────────────────────────────────────────────────────────────┐
│                    gz-sim 10.2.0                                 │
│  ┌──────────────────────┐   ┌──────────────────────────────────┐ │
│  │   tuba_ocean world   │   │        tuba_glider model         │ │
│  │  - graded buoyancy   │   │  - BuoyancyEngine plugin         │ │
│  │  - Coast Water (Fuel)│   │  - Hydrodynamics (Fossen)        │ │
│  │  - OceanFloor (Fuel) │   │  - LiftDrag ×2 (wings)          │ │
│  │  - 1 ms physics step │   │  - JointPositionController       │ │
│  └──────────────────────┘   │  - IMU + Altimeter sensors       │ │
└─────────────────────────────┴──────────────────────────────────-─┘
                                         │ gz-transport topics
                              ┌──────────▼──────────┐
                              │  gz_zephyr_bridge.py │
                              │  (UDP ↔ gz.transport) │
                              └──────────┬──────────┘
                                UDP 5555 ↓ sensors
                                UDP 5556 ↑ actuators
                              ┌──────────────────────┐
                              │  Zephyr RTOS (QEMU)  │
                              │  native_posix / arm  │
                              │  state machine:      │
                              │  IDLE→DESCEND→ASCEND │
                              │  →SURFACE→repeat     │
                              └──────────────────────┘
```

---

## 3. Ocean World

**File:** `worlds/tuba_ocean.sdf`

### 3.1 Fluid Model

| Zone | Condition | Density |
|------|-----------|---------|
| Above z=0 | Air | 1.2 kg/m³ |
| At and below z=0 | Seawater | 1025 kg/m³ |

The `gz::sim::systems::Buoyancy` plugin with `<graded_buoyancy>` implements this transition. The `<enable>tuba_glider</enable>` tag restricts computation to the glider, preventing numerical issues on large static mesh models.

### 3.2 World Contents

| Element | Source | Pose |
|---------|--------|------|
| Wave surface | Fuel: `Coast Water` | z=0 |
| Seafloor + shipwreck | Fuel: `OceanFloorShipwreck` | z=-50 |
| Glider | Local model | z=-5 |

### 3.3 System Plugins

`Physics`, `UserCommands`, `SceneBroadcaster`, `Contact`, `Sensors` (ogre2), `Imu`, `Altimeter`, `Buoyancy`

---

## 4. Glider Model

**File:** `models/tuba_glider/model.sdf`

### 4.1 Link Architecture

| Link | Type | Mass | Purpose |
|------|------|------|---------|
| `hull_link` | Cylinder (placeholder) | 59.9 kg | Main body, sensors |
| `wing_port_link` | Box 0.08×0.40×0.01 m | 0.2 kg | Port wing (LiftDrag) |
| `wing_stbd_link` | Box 0.08×0.40×0.01 m | 0.2 kg | Starboard wing (LiftDrag) |
| `mass_shifter_link` | Box 0.20×0.05×0.05 m | 4.0 kg | Sliding battery (pitch ctrl) |
| `tail_fin_link` | Box 0.10×0.005×0.15 m | 0.1 kg | Vertical stabiliser |

Total model mass: 64.4 kg. At neutral buoyancy (bladder = 0.002 m³):  
Net buoyancy force = 1025 × 0.06235 × 9.81 − 64.4 × 9.81 ≈ 0 N.

### 4.2 Joints

| Joint | Type | Axis | Range |
|-------|------|------|-------|
| `wing_port_joint` | fixed | — | — |
| `wing_stbd_joint` | fixed | — | — |
| `tail_fin_joint` | fixed | — | — |
| `mass_shifter_joint` | prismatic | X | ±0.15 m |

### 4.3 Plugins

| Plugin | Purpose |
|--------|---------|
| `gz::sim::systems::BuoyancyEngine` | Bladder volume → net vertical force |
| `gz::sim::systems::JointPositionController` | PID servo for mass_shifter_joint |
| `gz::sim::systems::Hydrodynamics` | Fossen added mass + quadratic drag |
| `gz::sim::systems::LiftDrag` ×2 | Hydrodynamic lift on each wing |
| `gz::sim::systems::PosePublisher` | Ground truth pose at 10 Hz |
| `gz::sim::systems::JointStatePublisher` | Joint telemetry |

### 4.4 Sensors

| Sensor | Topic | Rate | Message |
|--------|-------|------|---------|
| IMU | `/model/tuba_glider/imu` | 100 Hz | `gz.msgs.IMU` |
| Altimeter | `/model/tuba_glider/altimeter` | 10 Hz | `gz.msgs.Altimeter` |

---

## 5. Control Interface

### 5.1 Input Topics (commands to Gazebo)

| Topic | Message | Description |
|-------|---------|-------------|
| `/model/tuba_glider/buoyancy_engine` | `gz.msgs.Double` (m³) | Bladder volume setpoint |
| `/model/tuba_glider/joint/mass_shifter_joint/0/cmd_pos` | `gz.msgs.Double` (m) | Mass position setpoint |

### 5.2 Output Topics (telemetry from Gazebo)

| Topic | Message | Description |
|-------|---------|-------------|
| `/model/tuba_glider/buoyancy_engine/current_volume` | `gz.msgs.Double` | Actual bladder volume |
| `/model/tuba_glider/imu` | `gz.msgs.IMU` | Orientation, angular velocity, linear acceleration |
| `/model/tuba_glider/altimeter` | `gz.msgs.Altimeter` | `vertical_position = z` (depth = −z) |
| `/model/tuba_glider/pose` | `gz.msgs.Pose_V` | All link poses at 10 Hz |

### 5.3 Glide Cycle Commands

| Phase | Bladder | Mass Position | Expected Behaviour |
|-------|---------|---------------|-------------------|
| Dive | 0.001 m³ | +0.12 m | Sinks + forward glide, nose-down |
| Ascend | 0.003 m³ | −0.12 m | Rises + forward glide, nose-up |
| Surface | 0.003 m³ | 0.0 m | Levels at surface |

---

## 6. Hydrodynamics Parameters

Fossen (1994) model: τ = −M_A·ν̇ − C_A(ν)·ν − D(ν)·ν

| Parameter | Value | Physical Basis |
|-----------|-------|----------------|
| `xDotU` | −2.0 kg | Surge added mass (streamlined bow) |
| `yDotV`, `zDotW` | −40.0 kg | Cross-flow added mass |
| `mDotQ`, `nDotR` | −4.0 kg·m² | Rotational added inertia |
| `xUabsU` | −3.0 | Axial quadratic drag |
| `yVabsV`, `zWabsW` | −150.0 | Cross-flow quadratic drag |
| `mQabsQ`, `nRabsR` | −25.0 | Pitch/yaw quadratic drag |

Tuning file: `config/physics_params.yaml`

---

## 7. Zephyr SIL Integration

### 7.1 UDP Protocol

**SensorPacket** (bridge → Zephyr, 56 bytes, port 5555):

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | magic | u32 | 0xDEADBEEF |
| 4 | sim_time_ns | u64 | Simulation time (ns) |
| 12 | depth_m | f32 | Depth below surface (m) |
| 16 | pitch_rad | f32 | Body pitch (rad) |
| 20 | roll_rad | f32 | Body roll (rad) |
| 24 | yaw_rad | f32 | Body yaw (rad) |
| 28–40 | acc_xyz | 3×f32 | Linear acceleration (m/s²) |
| 40–52 | gyro_xyz | 3×f32 | Angular velocity (rad/s) |
| 52 | checksum | u32 | XOR of bytes 0–51 |

**ActuatorPacket** (Zephyr → bridge, 16 bytes, port 5556):

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | magic | u32 | 0xBEEFCAFE |
| 4 | bladder_vol_m3 | f32 | Volume setpoint (m³) |
| 8 | mass_pos_m | f32 | Position setpoint (m) |
| 12 | checksum | u32 | XOR of bytes 0–11 |

### 7.2 Zephyr State Machine

```
IDLE → (sensor_valid && depth > 1 m) → DESCEND
DESCEND → (depth >= 30 m) → ASCEND
ASCEND  → (depth <= 3 m)  → SURFACE
SURFACE → (30 s elapsed)  → DESCEND
```

### 7.3 Build Targets

| Board | Command | Notes |
|-------|---------|-------|
| `native_posix` | `west build -b native_posix` | Native Linux, simpler networking, recommended for dev |
| `qemu_cortex_m3` | `west build -b qemu_cortex_m3` | ARM Cortex-M3 fidelity, SLIRP networking, host at 10.0.2.2 |

---

## 8. Acceptance Criteria

| Phase | Criterion |
|-------|-----------|
| 1 | World loads, no CRITICAL errors, both Fuel models visible |
| 2 | `gz sdf --check` clean; neutral buoyancy holds depth within ±0.1 m |
| 3 | Glide angle 20°–40°; roll ≤ ±5°; horizontal travel per cycle > 50 m |
| 4 | `native_posix` SIL completes full sawtooth cycle; depth profile matches Phase 3 within ±2 m |

---

## 9. Open Items

1. **Real shell mesh** — replace cylinder placeholder with actual CAD mesh; update inertia values.
2. **Fuel model verification** — run `gz fuel download` for both models before first launch.
3. **Hydrodynamics tuning** — coefficients are analytical estimates; 1–2 iteration cycles expected.
4. **Zephyr SDK version** — `CONFIG_ETH_SMSC911X` must be available for `qemu_cortex_m3` target.
