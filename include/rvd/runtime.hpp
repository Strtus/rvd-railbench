#pragma once
#include "rvd/config.hpp"
#include <string>
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
  std::string log = "pfcr_rail_log.csv";
};


Options parse_options(int argc, char** argv);
int run(const Options& opt, const RailConfig& cfg);

}  // namespace rvd
