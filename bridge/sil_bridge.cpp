// sil_bridge.cpp — Gazebo <-> Tuba firmware bridge (software/hardware-in-the-loop)
//
// One process that sits between gz-sim and the glider firmware:
//
//   firmware (native_sim or ESP32)  --UDP sil_act_pkt-->   this bridge
//        ^                                                     |
//        |                                   integrates motor run time into
//        |                                   bladder volume / battery position,
//        |                                   publishes them to Gazebo
//        |                                                     v
//        +---UDP sil_sens_pkt--- pose from Gazebo -> pressure, attitude, GPS
//
// The firmware side only ever sees what the real hardware sees: H-bridge
// pin states going out, raw sensor registers coming in. Everything
// "physical" (stroke times, end stops, where the pressure port sits on the
// hull, water density) is configured here, in config/sil.cfg.
//
// Build: scripts/build_bridge.sh      Run: scripts/run_sil.sh (or directly)
//
//   bridge/build/sil_bridge [--config config/sil.cfg] [--set key=value ...]
//                           [--log run.csv] [--quiet]
//
// Console commands while running: status, leak on|off, reset, set key value, quit

#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/double.pb.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include "sil_protocol.h"

namespace {

// ---------------------------------------------------------------- config
struct Config {
  std::map<std::string, std::string> kv;

  std::string str(const std::string &k, const std::string &def) const {
    auto it = kv.find(k);
    return it == kv.end() ? def : it->second;
  }
  double num(const std::string &k, double def) const {
    auto it = kv.find(k);
    return it == kv.end() ? def : std::stod(it->second);
  }
  bool load(const std::string &path) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
      auto hash = line.find('#');
      if (hash != std::string::npos) line.erase(hash);
      std::istringstream ss(line);
      std::string k, v;
      if (ss >> k >> v) kv[k] = v;
    }
    return true;
  }
};

// ---------------------------------------------------------------- state
struct Pose {
  double x = 0, y = 0, z = 0;
  double qw = 1, qx = 0, qy = 0, qz = 0;
  double sim_time = 0;
  bool valid = false;
};

struct Actuator {
  double pos_s = 0;     // physical position, seconds of travel from the boot end
  double min_s = 0, max_s = 12;
  float last_run_s = 0; // firmware's cumulative run time at the last packet
  int8_t dir = 0;
  int32_t fw_pos_s = 0; // firmware's own belief (display)

  // Apply a firmware packet: move by the change in run time, clamped.
  void apply(float run_s, int8_t d, int32_t fw_pos, bool resync) {
    if (!resync) pos_s += (double)(run_s - last_run_s);
    pos_s = std::clamp(pos_s, min_s, max_s);
    last_run_s = run_s;
    dir = d;
    fw_pos_s = fw_pos;
  }
};

std::atomic<bool> g_running{true};
void on_signal(int) { g_running = false; }

double now_wall() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// Quaternion -> rotation matrix third row (world z of a body-frame offset).
void quat_to_euler(const Pose &p, double &roll, double &pitch, double &yaw) {
  double sinr = 2.0 * (p.qw * p.qx + p.qy * p.qz);
  double cosr = 1.0 - 2.0 * (p.qx * p.qx + p.qy * p.qy);
  roll = std::atan2(sinr, cosr);
  double sinp = 2.0 * (p.qw * p.qy - p.qz * p.qx);
  pitch = std::asin(std::clamp(sinp, -1.0, 1.0));
  double siny = 2.0 * (p.qw * p.qz + p.qx * p.qy);
  double cosy = 1.0 - 2.0 * (p.qy * p.qy + p.qz * p.qz);
  yaw = std::atan2(siny, cosy);
}

double world_z_of(const Pose &p, double bx, double by, double bz) {
  // Third row of R(q): R20 = 2(xz - wy), R21 = 2(yz + wx), R22 = 1 - 2(x^2 + y^2)
  double r20 = 2.0 * (p.qx * p.qz - p.qw * p.qy);
  double r21 = 2.0 * (p.qy * p.qz + p.qw * p.qx);
  double r22 = 1.0 - 2.0 * (p.qx * p.qx + p.qy * p.qy);
  return p.z + r20 * bx + r21 * by + r22 * bz;
}

}  // namespace

int main(int argc, char **argv) {
  // ---------------------------------------------------------- arguments
  std::string cfg_path = "config/sil.cfg", log_path;
  bool quiet = false;
  std::map<std::string, std::string> overrides;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&](const char *what) -> std::string {
      if (i + 1 >= argc) { std::cerr << a << " needs " << what << "\n"; std::exit(2); }
      return argv[++i];
    };
    if (a == "--config") cfg_path = next("a path");
    else if (a == "--log") log_path = next("a path");
    else if (a == "--quiet") quiet = true;
    else if (a == "--set") {
      std::string kv = next("key=value");
      auto eq = kv.find('=');
      if (eq == std::string::npos) { std::cerr << "--set needs key=value\n"; return 2; }
      overrides[kv.substr(0, eq)] = kv.substr(eq + 1);
    } else if (a == "-h" || a == "--help") {
      std::cout << "usage: sil_bridge [--config FILE] [--set key=value]... [--log FILE.csv] [--quiet]\n";
      return 0;
    } else { std::cerr << "unknown argument " << a << "\n"; return 2; }
  }

  Config cfg;
  if (!cfg.load(cfg_path)) {
    std::cerr << "[bridge] cannot read " << cfg_path << " (run from the tuba-sim root or pass --config)\n";
    return 1;
  }
  for (auto &kv : overrides) cfg.kv[kv.first] = kv.second;

  // gz-transport discovery on this machine only works reliably on loopback.
  setenv("GZ_IP", "127.0.0.1", 0);

  const std::string world = cfg.str("world", "tuba_ocean");
  const std::string model = cfg.str("model", "my_glider_v2");
  const std::string pose_topic = "/world/" + world + "/dynamic_pose/info";
  const std::string buoy_topic = cfg.str("buoyancy_topic", "/model/" + model + "/buoyancy_engine");
  const std::string batt_topic = cfg.str("battery_topic", "/" + model + "/battery_joint_pos");
  const std::string piston_topic = cfg.str("piston_topic", "");

  Actuator pump, pitch, roll;
  auto reload_limits = [&]() {
    pump.min_s = 0;  pump.max_s = cfg.num("pump_full_stroke_s", 12);
    pitch.min_s = 0; pitch.max_s = cfg.num("pitch_full_stroke_s", 12);
    roll.max_s = cfg.num("roll_limit_s", 10); roll.min_s = -roll.max_s;
  };
  reload_limits();

  // ---------------------------------------------------------- Gazebo
  gz::transport::Node node;
  Pose pose;
  std::mutex pose_mtx;
  std::function<void(const gz::msgs::Pose_V &)> on_pose = [&](const gz::msgs::Pose_V &msg) {
    for (const auto &p : msg.pose()) {
      if (p.name() != model) continue;
      std::lock_guard<std::mutex> lk(pose_mtx);
      pose.x = p.position().x(); pose.y = p.position().y(); pose.z = p.position().z();
      pose.qw = p.orientation().w(); pose.qx = p.orientation().x();
      pose.qy = p.orientation().y(); pose.qz = p.orientation().z();
      pose.sim_time = msg.header().stamp().sec() + msg.header().stamp().nsec() * 1e-9;
      pose.valid = true;
    }
  };
  if (!node.Subscribe(pose_topic, on_pose)) {
    std::cerr << "[bridge] failed to subscribe " << pose_topic << "\n";
    return 1;
  }
  auto pub_buoy = node.Advertise<gz::msgs::Double>(buoy_topic);
  auto pub_batt = node.Advertise<gz::msgs::Double>(batt_topic);
  gz::transport::Node::Publisher pub_piston;
  if (!piston_topic.empty()) pub_piston = node.Advertise<gz::msgs::Double>(piston_topic);

  // ---------------------------------------------------------- UDP
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_port = htons((uint16_t)cfg.num("listen_port", SIL_BRIDGE_PORT));
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  int one = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  if (bind(sock, (sockaddr *)&local, sizeof(local)) < 0) {
    perror("[bridge] bind");
    return 1;
  }
  sockaddr_in fw{};
  fw.sin_family = AF_INET;
  fw.sin_port = htons((uint16_t)cfg.num("fw_port", SIL_FW_PORT));
  inet_pton(AF_INET, cfg.str("fw_host", "127.0.0.1").c_str(), &fw.sin_addr);

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::ofstream log;
  if (!log_path.empty()) {
    log.open(log_path);
    log << "wall_s,sim_s,depth_m,pitch_deg,roll_deg,heading_deg,pump_s,volume_l,pitch_s,battery_m,"
           "roll_s,pump_dir,pitch_dir,limit_up,limit_down,gps_fix,fw_pump_s,fw_pitch_s\n";
  }

  std::cout << "[bridge] gz: " << pose_topic << " -> " << buoy_topic << ", " << batt_topic << "\n"
            << "[bridge] firmware: " << cfg.str("fw_host", "127.0.0.1") << ":" << (int)cfg.num("fw_port", SIL_FW_PORT)
            << "  (listening on " << (int)cfg.num("listen_port", SIL_BRIDGE_PORT) << ")\n"
            << "[bridge] commands: status | leak on|off | reset | set key value | quit\n";

  // ---------------------------------------------------------- main loop
  const double period = 1.0 / cfg.num("rate_hz", 20);
  uint32_t seq = 0, fw_boot_id = 0;
  bool fw_seen = false, leak = false, stdin_open = true;
  double last_fw_rx = 0, last_status = 0, last_sim_time = -1, last_sim_change = now_wall();
  double next_tick = now_wall();
  double internal_pa_extra = 0;   // leak: hull pressure creeps up
  std::string stdin_buf;

  while (g_running) {
    // --- 1. receive actuator packets from the firmware --------------------
    for (;;) {
      sil_act_pkt act{};
      sockaddr_in from{};
      socklen_t fl = sizeof(from);
      ssize_t n = recvfrom(sock, &act, sizeof(act), MSG_DONTWAIT, (sockaddr *)&from, &fl);
      if (n <= 0) break;
      if (n != (ssize_t)sizeof(act) || act.magic != SIL_MAGIC_ACT ||
          act.crc != sil_fnv1a(&act, sizeof(act) - sizeof(act.crc)))
        continue;
      bool resync = !fw_seen || act.boot_id != fw_boot_id;
      if (resync && fw_seen)
        std::cout << "\n[bridge] firmware rebooted (new boot id); dead-reckoning restarts at 0, "
                     "physical positions kept\n";
      if (!fw_seen) std::cout << "\n[bridge] firmware connected from " << inet_ntoa(from.sin_addr) << "\n";
      fw_seen = true;
      fw_boot_id = act.boot_id;
      last_fw_rx = now_wall();
      fw = from;  // answer whoever talks to us (ESP32 over WiFi included)
      pump.apply(act.pump_run_s, act.pump_dir, act.pump_pos_s, resync);
      pitch.apply(act.pitch_run_s, act.pitch_dir, act.pitch_pos_s, resync);
      roll.apply(act.roll_run_s, act.roll_dir, act.roll_pos_s, resync);
    }
    bool fw_link = fw_seen && (now_wall() - last_fw_rx) < 2.0;

    // --- 2. console commands ----------------------------------------------
    if (stdin_open) {
      fd_set rfds; FD_ZERO(&rfds); FD_SET(0, &rfds);
      timeval tv{0, 0};
      if (select(1, &rfds, nullptr, nullptr, &tv) > 0) {
        char buf[256];
        ssize_t n = read(0, buf, sizeof(buf));
        if (n > 0) stdin_buf.append(buf, n);
        else stdin_open = false;   // EOF (e.g. started from a script): stop polling
      }
      size_t nl;
      while ((nl = stdin_buf.find('\n')) != std::string::npos) {
        std::string line = stdin_buf.substr(0, nl);
        stdin_buf.erase(0, nl + 1);
        std::istringstream ss(line);
        std::string cmd, a, b;
        ss >> cmd >> a >> b;
        if (cmd == "quit" || cmd == "exit") g_running = false;
        else if (cmd == "leak") { leak = (a == "on"); std::cout << "[bridge] leak " << (leak ? "ON" : "off") << "\n"; }
        else if (cmd == "reset") { pump.pos_s = pitch.pos_s = roll.pos_s = 0; std::cout << "[bridge] actuators back to boot positions\n"; }
        else if (cmd == "set" && !a.empty() && !b.empty()) { cfg.kv[a] = b; reload_limits(); std::cout << "[bridge] " << a << " = " << b << "\n"; }
        else if (cmd == "status") last_status = 0;
        else if (!cmd.empty()) std::cout << "[bridge] commands: status | leak on|off | reset | set key value | quit\n";
      }
    }

    // --- 3. physics mapping + Gazebo commands ------------------------------
    const double pump_rate_l = (cfg.num("pump_max_l", 1.2) - cfg.num("pump_min_l", 0.0)) / std::max(1e-6, pump.max_s);
    const double pitch_rate_m = (cfg.num("pitch_max_m", 0.14) - cfg.num("pitch_min_m", -0.05)) / std::max(1e-6, pitch.max_s);
    const double volume_l = cfg.num("pump_min_l", 0.0) + pump.pos_s * pump_rate_l;
    const double battery_m = cfg.num("pitch_min_m", -0.05) + pitch.pos_s * pitch_rate_m;
    const double margin = cfg.num("limit_margin_s", 0.15);
    const bool limit_up = pitch.pos_s >= pitch.max_s - margin;
    const bool limit_down = pitch.pos_s <= pitch.min_s + margin;

    if (fw_seen) {  // without a firmware, leave the world at its own defaults
      gz::msgs::Double m;
      m.set_data(volume_l / 1000.0);
      pub_buoy.Publish(m);
      m.set_data(battery_m);
      pub_batt.Publish(m);
      if (!piston_topic.empty()) {
        double span = std::max(1e-6, cfg.num("pump_max_l", 1.2) - cfg.num("pump_min_l", 0.0));
        m.set_data(cfg.num("piston_max_m", 0.05) * (volume_l - cfg.num("pump_min_l", 0.0)) / span);
        pub_piston.Publish(m);
      }
    }

    // --- 4. sensors from the pose ------------------------------------------
    Pose p;
    { std::lock_guard<std::mutex> lk(pose_mtx); p = pose; }
    if (p.sim_time != last_sim_time) { last_sim_time = p.sim_time; last_sim_change = now_wall(); }
    const bool paused = !p.valid || (now_wall() - last_sim_change) > 0.5;

    double rollr = 0, pitchr = 0, yawr = 0;
    quat_to_euler(p, rollr, pitchr, yawr);
    const double surface_z = cfg.num("surface_z", 20.0);
    const double z_sensor = world_z_of(p, cfg.num("ms5837_x", -0.5387), cfg.num("ms5837_y", 0), cfg.num("ms5837_z", 0));
    const double z_centre = world_z_of(p, cfg.num("hull_centre_x", -0.5387), 0, 0);
    const double z_tail = world_z_of(p, cfg.num("tail_end_x", -1.1887), 0, 0);
    const double depth_sensor = std::max(0.0, surface_z - z_sensor);
    const double depth_centre = surface_z - z_centre;
    const bool gps_fix = p.valid && (z_tail - surface_z) >= cfg.num("gps_margin_m", 0.0);

    if (leak) internal_pa_extra += 30.0 * period; else internal_pa_extra = 0;

    const double lat0 = cfg.num("gps_lat0", 36.8), lon0 = cfg.num("gps_lon0", -121.9);
    double heading = 90.0 - yawr * 180.0 / M_PI + cfg.num("heading_offset_deg", 0);
    heading = std::fmod(std::fmod(heading, 360.0) + 360.0, 360.0);

    sil_sens_pkt s{};
    s.magic = SIL_MAGIC_SENS;
    s.seq = ++seq;
    s.flags = (gps_fix ? SIL_F_GPS_FIX : 0) | (limit_up ? SIL_F_LIMIT_UP : 0) |
              (limit_down ? SIL_F_LIMIT_DOWN : 0) | (paused ? SIL_F_SIM_PAUSED : 0) | (leak ? SIL_F_LEAK : 0);
    s.sim_time_s = p.sim_time;
    s.utc_unix_s = (int64_t)std::time(nullptr);
    s.ext_pressure_pa = (float)(cfg.num("atm_pressure_pa", 101325) + cfg.num("water_density", 1025) * 9.80665 * depth_sensor);
    s.water_temp_c = (float)cfg.num("water_temp_c", 15);
    s.int_pressure_pa = (float)(cfg.num("internal_pressure_pa", 101300) + internal_pa_extra);
    s.int_temp_c = (float)cfg.num("internal_temp_c", 25);
    s.heading_deg = (float)heading;
    s.pitch_deg = (float)(-pitchr * 180.0 / M_PI);   // HMC6343: + nose-up; gz: + nose-down
    s.roll_deg = (float)(rollr * 180.0 / M_PI);
    s.lat_deg = lat0 + p.y / 111320.0;
    s.lon_deg = lon0 + p.x / (111320.0 * std::cos(lat0 * M_PI / 180.0));
    s.depth_m = (float)depth_centre;
    s.leak_mv = (float)cfg.num(leak ? "leak_wet_mv" : "leak_dry_mv", leak ? 400 : 3000);
    s.crc = sil_fnv1a(&s, sizeof(s) - sizeof(s.crc));
    if (p.valid) sendto(sock, &s, sizeof(s), 0, (sockaddr *)&fw, sizeof(fw));

    // --- 5. status / log ----------------------------------------------------
    if (!quiet && now_wall() - last_status >= 1.0) {
      last_status = now_wall();
      char line[400];
      std::snprintf(line, sizeof(line),
          "[bridge] t=%7.1fs%s depth=%5.2fm pitch=%+6.1f hdg=%5.1f | pump %5.2fs=%.2fL d%+d | "
          "pitch %5.2fs=%+.3fm d%+d%s%s | roll %+5.2fs | gps %s | fw %s\n",
          p.sim_time, paused ? " PAUSED" : "", depth_centre, -s.pitch_deg, heading,
          pump.pos_s, volume_l, pump.dir, pitch.pos_s, battery_m, pitch.dir,
          limit_up ? " UP" : "", limit_down ? " DOWN" : "", roll.pos_s,
          gps_fix ? "fix" : "---", fw_link ? "ok" : (fw_seen ? "LOST" : "waiting"));
      std::cout << line << std::flush;
    }
    if (log) {
      log << now_wall() << ',' << p.sim_time << ',' << depth_centre << ',' << -s.pitch_deg << ','
          << s.roll_deg << ',' << heading << ',' << pump.pos_s << ',' << volume_l << ',' << pitch.pos_s << ','
          << battery_m << ',' << roll.pos_s << ',' << (int)pump.dir << ',' << (int)pitch.dir << ','
          << limit_up << ',' << limit_down << ',' << gps_fix << ',' << pump.fw_pos_s << ',' << pitch.fw_pos_s << '\n';
    }

    next_tick += period;
    double sleep_s = next_tick - now_wall();
    if (sleep_s > 0) std::this_thread::sleep_for(std::chrono::duration<double>(sleep_s));
    else next_tick = now_wall();
  }

  close(sock);
  std::cout << "[bridge] stopped\n";
  return 0;
}
