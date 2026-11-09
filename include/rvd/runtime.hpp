#pragma once
#include <cstdint>
#include <string>
#include <vector>

#include "rvd/config.hpp"
namespace rvd {
std::string find_model_dir();
}

namespace rvd {
using pfcr::Vec3;
struct Options {
  std::string model_dir = find_model_dir();
  bool onnx = false, lab = false, fast = false;
  bool open_loop = false;
  int max_steps = -1;
  double noise = 0.0005;
  double sim_latency = 0.010;
  int sim_freeze_at = -1;
  Vec3 start_hill{18.0, 0.3, -0.2};
  RailConfig rail;
  bool bounded = false;
  // PLC positioning semantics: move at the commanded speed, stop at the target, retarget while moving.
  bool plc = false;
  Vec3 plc_accel{0.2, 0.2, 0.2};  // [m/s^2] plant acceleration, independent of the software limit
  double plc_delay = 0.03;        // [s] from send to the PLC applying a command
  std::uint32_t seed = 12345;
  int sim_fail_read_at = -1, sim_fail_send_at = -1;
  int sim_repeat_pose_at = -1;  // motion capture repeats the last pose with fresh stamps
  bool sim_encoder = true;      // report axis encoder positions alongside motion capture
  Vec3 sim_bias{};
  std::string sim_trace;  // optional plant trace CSV (bounded and PLC modes)
  int bridge_port = 0;    // rail I/O relayed by RVDLAB on this loopback port instead of the simulator
  std::string experiment_dir;
  std::vector<std::string> config_files;
  std::string log = "pfcr_rail_log.csv";
};

void load_config(const std::string& path, Options& options);
void validate(const Options& options);
std::string configuration_snapshot(const Options& options);
Options parse_options(int argc, char** argv);
int run(const Options& opt);

}  // namespace rvd
