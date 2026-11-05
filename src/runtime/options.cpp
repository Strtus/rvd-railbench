#include "rvd/runtime.hpp"
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
namespace rvd {
using namespace pfcr;
std::string find_model_dir() {
  const std::string rel = "model/" + std::string(params::kModelId);
  for (const char* up : {"", "../", "../../", "../../../"}) {
    const std::string d = std::string(up) + rel;
    if (std::FILE* f = std::fopen((d + "/pfcr_policy_weights.bin").c_str(), "rb")) { std::fclose(f); return d; }
  }
  return rel;
}

Options parse_options(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--onnx") opt.onnx = true;
    else if (a == "--lab") opt.lab = true;
    else if (a == "--open-loop") opt.open_loop = true;
    else if (a == "--fast") opt.fast = true;
    else if (a == "--model-dir" && i + 1 < argc) opt.model_dir = argv[++i];
    else if (a == "--log" && i + 1 < argc) opt.log = argv[++i];
    else if (a == "--noise" && i + 1 < argc) opt.noise = std::atof(argv[++i]);
    else if (a == "--max-steps" && i + 1 < argc) opt.max_steps = std::atoi(argv[++i]);
    else if (a == "--sim-latency" && i + 1 < argc) opt.sim_latency = std::atof(argv[++i]);
    else if (a == "--sim-freeze-at" && i + 1 < argc) opt.sim_freeze_at = std::atoi(argv[++i]);
    else if (a == "--start" && i + 3 < argc) { for (int j = 0; j < 3; ++j) opt.start_hill[j] = std::atof(argv[++i]); }
    else {
      std::printf("usage: rail_main [--onnx] [--lab] [--open-loop] [--fast] [--max-steps N] [--model-dir DIR] "
                  "[--log FILE] [--noise M] [--start X Y Z] [--sim-latency S] [--sim-freeze-at K]\n");
      if (a == "--help") throw std::invalid_argument("help");
      throw std::invalid_argument("invalid command line");
    }
  }
  return opt;
}
}
