#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include "rvd/runtime.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace rvd {
namespace {
std::string external_cli_name(std::string name) {
#ifdef _WIN32
  // A narrow Windows main() receives arguments in the system code page. Metadata is UTF-8.
  if (!name.empty()) {
    const auto fail = [] { return std::invalid_argument("cannot encode the external name as UTF-8"); };
    const auto bytes = static_cast<int>(name.size());
    const int length = MultiByteToWideChar(CP_ACP, MB_ERR_INVALID_CHARS, name.data(), bytes, nullptr, 0);
    if (length <= 0) throw fail();
    std::wstring wide(static_cast<std::size_t>(length), L'\0');
    if (MultiByteToWideChar(CP_ACP, MB_ERR_INVALID_CHARS, name.data(), bytes, wide.data(), length) != length)
      throw fail();
    const int utf8_length =
        WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, wide.data(), length, nullptr, 0, nullptr, nullptr);
    if (utf8_length <= 0) throw fail();
    name.resize(static_cast<std::size_t>(utf8_length));
    if (WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, wide.data(), length, name.data(), utf8_length,
                            nullptr, nullptr) != utf8_length)
      throw fail();
  }
#endif
  return name;
}
}  // namespace
std::string find_model_dir(const std::string& model_id) {
  for (const char* up : {"", "../", "../../", "../../../"}) {
    for (const char* root : {"policies/", "model/"}) {
      const auto d = std::string(up) + root + model_id;
      if (std::filesystem::is_regular_file(d + "/pfcr_policy_weights.bin")) return d;
    }
  }
  return "policies/" + model_id;
}
Options parse_options(int argc, char** argv) {
  Options opt;
  bool log_requested = false, external_name_requested = false;
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
    else if (a == "--bounded") {
      opt.bounded = true;
      opt.plc = false;
    } else if (a == "--plc")
      opt.bounded = opt.plc = true;
    else if (a == "--command-mode") {
      const auto mode = next(i);
      if (mode != "arrive" && mode != "lead")
        throw std::invalid_argument("command mode must be arrive or lead");
      opt.rail.command_mode = mode == "lead" ? CommandMode::Lead : CommandMode::Arrive;
    } else if (a == "--sim-trace")
      opt.sim_trace = next(i);
    else if (a == "--bridge")
      opt.bridge_port = integer(i);
    else if (a == "--external-port") {
      opt.external_port = integer(i);
      if (opt.external_port <= 0) throw std::invalid_argument("external port must be 1..65535");
    } else if (a == "--external-name") {
      opt.external_name = external_cli_name(next(i));
      external_name_requested = true;
    } else if (a == "--sim-repeat-pose-at")
      opt.sim_repeat_pose_at = integer(i);
    else if (a == "--model-dir")
      opt.model_dir = next(i);
    else if (a == "--log") {
      opt.log = next(i);
      log_requested = true;
    } else if (a == "--experiment-dir")
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
          "usage: rail_main [--config FILE]... [--onnx] [--lab] [--open-loop] [--fast] [--bounded | --plc]\n"
          "  [--command-mode arrive|lead] [--max-steps N] [--model-dir DIR] [--log FILE]\n"
          "  [--experiment-dir NEW_DIR] [--noise M] [--start X Y Z] [--sim-latency S] [--sim-freeze-at K]\n"
          "  [--sim-repeat-pose-at K] [--sim-fail-read-at K] [--sim-fail-send-at K] [--seed UINT32]\n"
          "  [--sim-trace FILE] [--bridge PORT] [--external-port PORT [--external-name NAME]]");
      throw std::invalid_argument("help");
    } else
      throw std::invalid_argument("unknown argument: " + a);
  }
  // Compatibility with the original default model directory after reorganization.
  if (opt.external_port > 0) {
    if (!log_requested) opt.log.clear();
    if (!opt.model_dir.empty()) throw std::invalid_argument("--model-dir applies only to PFCR mode");
  } else {
    if (external_name_requested) throw std::invalid_argument("--external-name requires --external-port");
    const std::string legacy = "model/" + opt.rail.model_id;
    if (opt.model_dir.empty() || (opt.model_dir == legacy && !std::filesystem::exists(legacy)))
      opt.model_dir = find_model_dir(opt.rail.model_id);
  }
  validate(opt);
  return opt;
}
}  // namespace rvd
