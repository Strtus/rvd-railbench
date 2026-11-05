#pragma once
#include "rvd/config.hpp"
#include <string>
#include <cstdint>
#include <vector>
namespace rvd { std::string find_model_dir(); }

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
  std::uint32_t seed = 12345;
  int sim_fail_read_at = -1, sim_fail_send_at = -1;
  Vec3 sim_bias{};
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
