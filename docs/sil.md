# Software-in-the-loop: the real firmware flying the Gazebo glider

The [tuba-firmware](https://github.com/e-abdi/tuba-firmware) (Zephyr, ESP32)
can be built as a Linux executable and connected to this simulation. The
firmware's menus, parameters, missions and safety envelope then run exactly
as on the glider, but the pump and the pitch mass move the Gazebo model, and
the depth, compass and GPS it reads come from the model's pose.

```
 ┌────────────────────────────┐  UDP 5600/5601   ┌──────────────────────┐   gz-transport   ┌──────────┐
 │ tuba-firmware (native_sim) │ ───────────────▶ │ bridge/sil_bridge    │ ───────────────▶ │  gz sim  │
 │  menus · deploy · params   │  H-bridge pins   │  stroke model        │ bladder, battery │ my_glider│
 │  hw_*.c drivers UNCHANGED  │ ◀─────────────── │  end stops / limits  │ ◀─────────────── │   _v2    │
 │  on emulated GPIO/I2C/ADC  │  pressure, HPR,  │  sensor placement    │      pose        │          │
 └────────────────────────────┘  GPS, leak, ...   └──────────────────────┘                  └──────────┘
```

## What is simulated, and how faithfully

| Firmware driver | Real part | In the simulator |
|---|---|---|
| `hw_pump.c` | H-bridge on GPIO18/19 | same pins on `gpio_emul`; run time integrated by the bridge into bladder volume (`pump_full_stroke_s` ↔ 0–1.2 L) |
| `hw_motors.c` pitch | H-bridge on GPIO27/14 | battery joint, `pitch_full_stroke_s` ↔ −0.05…0.14 m |
| `hw_motors.c` roll | H-bridge on GPIO25/26 | position tracked and shown; the Gazebo model has no roll mass yet |
| `hw_limit_switches.c` | GPIO32/33 active-low | asserted by the bridge when the battery is within `limit_margin_s` of an end stop |
| `hw_ms5837.c` | I2C 0x76 | emulated chip with PROM + CRC; raw D1/D2 solved so the firmware's own math gives `atm + ρ g · depth(pressure port)` |
| `hw_bmp180.c` | I2C 0x77 | emulated chip, constant hull pressure (`leak on` makes it creep up) |
| `hw_hmc6343.c` | I2C 0x19 | emulated: heading from yaw, pitch/roll from the pose, EEPROM orientation byte |
| `hw_gps.c` | u-blox DDC 0x42 | emulated: NMEA RMC/GGA once a second, status `A` only while the antenna (tail end) is above the surface; UTC from the host clock |
| `hw_tsys01.c` | I2C 0x77 on `i2c_temp` | emulated on a second emulated bus, constant `water_temp_c` |
| `hw_leak.c` | ADC1 ch6 | `adc_emul`; `leak on` on the bridge console drops it below any threshold |
| parameters (NVS) | ESP32 flash | Zephyr flash simulator, persisted in `~/.tuba_sil_flash.bin` |
| WiFi AP / telnet, OTA, MCUboot | — | not built; console is the terminal |

The firmware sources are untouched: the simulator is a board overlay
(`boards/native_sim_native_64.overlay`), a config (`prj_sil.conf`) and the
`src/sil/` directory in the firmware repo (link thread + chip emulators).

## Conventions that matter

* **Pump:** firmware position 0 = bladder fully *in* (0 L, −5 N, the glider
  sinks) at boot. `+` seconds = pumping *out* = more buoyant. 0.1 L/s with the
  default 12 s stroke: 5 s neutral, 6 s surface float (+1 N), 4 s dive (−1 N).
* **Pitch:** position 0 = battery fully *aft* (~35° nose-up submerged).
  `+` = forward = nose-down. ~2 s level, 4 s ≈ 37° down, 12 s ≈ 75° down
  (surface attitude: antenna out). The limit switches sit at 0 and 12 s;
  the firmware backs off 1 s whenever one is pressed, so "climb pitch 0"
  actually settles at ~1 s (≈18° nose-up) and the boot position reads 1 s.
* Because the firmware's mission parameters are unsigned, the boot position
  must be the "least" end of each actuator. On the real glider this means:
  power up with the pump fully in and the battery fully aft, or accept that
  mission targets can only reach positions on the `+` side of wherever the
  actuators were at boot.
* Depth from the MS5837 uses the firmware's 1025 kg/m³; the bridge computes
  pressure with the same density (`water_density` in `config/sil.cfg`).

**Firmware parameter set that flies in this world** (PARAMETERS menu, then `s`):

| # | Parameter | Value |
|---|---|---|
| 1 | Dive depth | 5–10 m (seafloor is 170 m down) |
| 4 / 5 / 6 | Dive / Start / Climb pump | 4 / 6 / 6 s |
| 7 / 9 / a | Start / Dive / Climb pitch | 11 / 4 / 0 s |
| h | T_MISSION | 20 min (a 10 m cycle takes ~3 min) |
| k / l | Pump / Pitch full stroke | 12 / 12 s (must match `config/sil.cfg`) |

Inconsistency worth knowing in the firmware defaults: `dive_pump_s = 3`,
`start/climb_pump_s = 0` imply `+` = *less* buoyant, while the recovery
manoeuvre drives the pump `+` for a full stroke and calls that "fully OUT,
maximum buoyancy". The simulator follows the recovery code (`+` = buoyant);
with the stock defaults a deploy would therefore never sink.

## Gazebo on this machine

The Ubuntu 26.04 upgrade disabled the packages.osrfoundation.org source and
removed the `gz sim` command/GUI and `python3-gz-transport15`; only the
headless server (`/usr/libexec/gz/sim10/gz-sim-server`) survived.
`scripts/run_sim.sh` and `scripts/test_sil.py` fall back to it. To get the
GUI and the Python mission script back: re-enable
`/etc/apt/sources.list.d/gazebo-stable.list.disabled` (noble packages still
install) and `sudo apt install gz-sim10 python3-gz-transport15`.

## Running it

```bash
# once
bash scripts/build_bridge.sh                 # needs libgz-transport15-dev, libgz-msgs12-dev
cd ~/zephyrproject/tuba && ./sil_build.sh    # the firmware repo inside the Zephyr workspace

# every time (three terminals)
bash scripts/run_sim.sh                      # 1: Gazebo, press Play
bash scripts/run_sil.sh                      # 2: bridge; prints a status line every second
~/zephyrproject/tuba/sil_run.sh              # 3: the glider console (press ENTER for the menu)
```

Start the bridge before the firmware (or before using the sensors): without
it the firmware sees empty I2C buses, exactly like unplugged sensors.
Nothing is sent to Gazebo until a firmware connects, so the glider keeps
floating at the world's defaults until then; the first thing the firmware
does is put both actuators at their boot ends (0 L / battery aft), so the
glider sinks until you pump: hardware test → pump → `6`, then pitch → `10`.

Bridge console commands: `status`, `leak on|off`, `reset` (actuators back to
their boot ends), `set key value` (any `config/sil.cfg` key, live), `quit`.
`--log file.csv` records pose, actuator positions and the firmware's own
dead-reckoned positions at 20 Hz.

### Automated check

```bash
python3 scripts/test_sil.py              # headless gz + bridge + firmware, ~6 min
python3 scripts/test_sil.py --no-mission # sensors and actuators only
```

Walks the menus with `pexpect`: sensor scan, pump/pitch jogs, compass, GPS,
pressure stream, then a full deploy (dive to 5 m, climb, surface, stop on
ENTER). Runs in its own `GZ_PARTITION`, so an open Gazebo is unaffected.

## Hardware-in-the-loop (next step)

The link is plain UDP and the bridge answers whichever address talks to it,
so the same bridge serves an ESP32: join the `Tuba-Glider` AP, run
`bash scripts/run_sil.sh --set fw_host=192.168.4.1`. What remains on the
firmware side is building `src/sil/` for the ESP32 with per-device switches
(real MS5837 vs simulated, real pump vs simulated), so any mix of real parts
and simulated ones can be tested with the glider in Gazebo. The packet
format already reserves `flags` for those switches.
