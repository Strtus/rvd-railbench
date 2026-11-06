#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include "rvd/runtime.hpp"
namespace rvd {
std::string find_model_dir() {
  for (const char* up : {"", "../", "../../", "../../../"}) {
    for (const char* root : {"policies/", "model/"}) {
      const auto d = std::string(up) + root + pfcr::params::kModelId;
      if (std::filesystem::is_regular_file(d + "/pfcr_policy_weights.bin")) return d;
    }
  }
  return "policies/" + std::string(pfcr::params::kModelId);
}
Options parse_options(int argc, char** argv) {
  Options opt;
  // Config files are applied in order; explicit CLI values always win.
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--config") {
      if (++i >= argc) throw std::invalid_argument("--config requires a path");
      load_config(argv[i], opt);
    }
  }
  auto next = [&](int& i) -> std::string {
    if (++i >= argc) throw std::invalid_argument("missing option value");
    return argv[i];
  };
  auto number = [&](int& i) {
    const auto v = next(i);
    size_t used = 0;
    const double n = std::stod(v, &used);
    if (used != v.size() || !std::isfinite(n)) throw std::invalid_argument("expected finite number: " + v);
    return n;
  };
  auto integer = [&](int& i) {
    const double n = number(i);
    if (n < -1 || n > std::numeric_limits<int>::max() || n != std::floor(n))
      throw std::invalid_argument("invalid integer argument");
    return static_cast<int>(n);
  };
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--config") {
      next(i);
    } else if (a == "--onnx")
      opt.onnx = true;
    else if (a == "--lab")
      opt.lab = true;
    else if (a == "--open-loop")
      opt.open_loop = true;
    else if (a == "--fast")
      opt.fast = true;
    else if (a == "--bounded")
      opt.bounded = true;
    else if (a == "--model-dir")
      opt.model_dir = next(i);
    else if (a == "--log")
      opt.log = next(i);
    else if (a == "--experiment-dir")
      opt.experiment_dir = next(i);
    else if (a == "--noise")
      opt.noise = number(i);
    else if (a == "--max-steps")
      opt.max_steps = integer(i);
    else if (a == "--sim-latency")
      opt.sim_latency = number(i);
    else if (a == "--sim-freeze-at")
      opt.sim_freeze_at = integer(i);
    else if (a == "--sim-fail-read-at")
      opt.sim_fail_read_at = integer(i);
    else if (a == "--sim-fail-send-at")
      opt.sim_fail_send_at = integer(i);
    else if (a == "--seed") {
      const double n = number(i);
      if (n < 0 || n > UINT32_MAX || n != std::floor(n)) throw std::invalid_argument("seed must be uint32");
      opt.seed = static_cast<std::uint32_t>(n);
    } else if (a == "--start") {
      for (int j = 0; j < 3; ++j) opt.start_hill[j] = number(i);
    } else if (a == "--help") {
      std::puts(
          "usage: rail_main [--config FILE]... [--onnx] [--lab] [--open-loop] [--fast] [--bounded]\n"
          "  [--max-steps N] [--model-dir DIR] [--log FILE] [--experiment-dir NEW_DIR]\n"
          "  [--noise M] [--start X Y Z] [--sim-latency S] [--sim-freeze-at K]\n"
          "  [--sim-fail-read-at K] [--sim-fail-send-at K] [--seed UINT32]");
      throw std::invalid_argument("help");
    } else
      throw std::invalid_argument("unknown argument: " + a);
  }
  // Compatibility with the original default model directory after reorganization.
  const std::string legacy = "model/" + std::string(pfcr::params::kModelId);
  if (opt.model_dir == legacy && !std::filesystem::exists(legacy)) opt.model_dir = find_model_dir();
  validate(opt);
  return opt;
}
}  // namespace rvd
