#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

#include "rvd/frames.hpp"
#include "rvd/runtime.hpp"

namespace rvd {
namespace {
std::string trim(const std::string& s) {
  const auto a = s.find_first_not_of(" \t\r\n");
  return a == std::string::npos ? "" : s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}
double number(const std::string& s) {
  size_t n = 0;
  const double v = std::stod(s, &n);
  if (n != s.size() || !std::isfinite(v)) throw std::invalid_argument("expected finite number: " + s);
  return v;
}
bool boolean(const std::string& s) {
  if (s == "true") return true;
  if (s == "false") return false;
  throw std::invalid_argument("expected true or false: " + s);
}
int integer(const std::string& s) {
  const double v = number(s);
  if (v != std::floor(v) || v < -1 || v > std::numeric_limits<int>::max())
    throw std::invalid_argument("invalid integer: " + s);
  return static_cast<int>(v);
}
template <class T>
void triple(const std::string& s, T& a) {
  std::istringstream in(s);
  std::string cell;
  for (int i = 0; i < 3; ++i) {
    if (!std::getline(in, cell, ',')) throw std::invalid_argument("expected three comma-separated values");
    a[i] = number(trim(cell));
  }
  if (std::getline(in, cell, ',') || (!s.empty() && s.back() == ','))
    throw std::invalid_argument("expected exactly three values");
}
void require(bool condition, const char* message) {
  if (!condition) throw std::invalid_argument(message);
}
}  // namespace

void load_config(const std::string& path, Options& o) {
  std::ifstream in(path);
  if (!in) throw std::invalid_argument("cannot open configuration: " + path);
  std::set<std::string> seen;
  std::string section, line;
  int lineno = 0;
  while (std::getline(in, line)) {
    ++lineno;
    line = trim(line);
    if (line.empty() || line[0] == '#' || line[0] == ';') continue;
    try {
      if (line.front() == '[' && line.back() == ']') {
        section = line.substr(1, line.size() - 2);
        require(section == "rail" || section == "mapping" || section == "controller" ||
                    section == "simulation" || section == "experiment",
                "unknown configuration section");
        continue;
      }
      const auto eq = line.find('=');
      require(eq != std::string::npos, "expected key = value");
      const auto key = section + "." + trim(line.substr(0, eq));
      const auto val = trim(line.substr(eq + 1));
      require(!val.empty(), "empty configuration value");
      require(seen.insert(key).second, "duplicate configuration key");
      auto& c = o.rail;
      if (key == "rail.minimum_m")
        triple(val, c.rail_min);
      else if (key == "rail.maximum_m")
        triple(val, c.rail_max);
      else if (key == "rail.velocity_m_s")
        triple(val, c.rail_vmax);
      else if (key == "rail.acceleration_m_s2")
        triple(val, c.rail_amax);
      else if (key == "rail.measurement_age_s")
        c.max_measurement_age_s = number(val);
      else if (key == "rail.measurement_late_s")
        c.max_measurement_late_s = number(val);
      else if (key == "rail.clock_skew_s")
        c.max_clock_skew_s = number(val);
      else if (key == "rail.sample_jitter_s")
        c.max_sample_jitter_s = number(val);
      else if (key == "rail.communication_timeout_s")
        c.communication_timeout_s = number(val);
      else if (key == "rail.tracking_error_m")
        c.max_track_err_rail_m = number(val);
      else if (key == "rail.sensor_disagreement_m")
        c.max_sensor_disagreement_m = number(val);
      else if (key == "rail.compensate_latency")
        c.compensate_latency = boolean(val);
      else if (key == "rail.command_mode") {
        require(val == "arrive" || val == "lead", "command mode must be arrive or lead");
        c.command_mode = val == "lead" ? CommandMode::Lead : CommandMode::Arrive;
      } else if (key == "rail.lead_s")
        c.lead_s = number(val);
      else if (key == "rail.tracking_gain_per_s")
        c.tracking_gain_per_s = number(val);
      else if (key == "mapping.axis") {
        Vec3 axes{};
        triple(val, axes);
        for (int i = 0; i < 3; ++i) {
          require(axes[i] >= 0 && axes[i] <= 2 && axes[i] == std::floor(axes[i]), "axis must be 0, 1 or 2");
          c.rail_axis[i] = static_cast<int>(axes[i]);
        }
      } else if (key == "mapping.sign")
        triple(val, c.sign);
      else if (key == "mapping.scale")
        triple(val, c.scale);
      else if (key == "mapping.target_m")
        triple(val, c.target_rail);
      else if (key == "controller.type")
        require(val == "pfcr-dqp", "unsupported controller");
      else if (key == "controller.model_id") {
        require(pfcr::find_model(val) != nullptr, "incompatible policy artifact");
        c.model_id = val;
      } else if (key == "controller.period_s")
        c.period_s = number(val);
      else if (key == "controller.backend") {
        require(val == "builtin" || val == "onnx", "unsupported policy backend");
        o.onnx = val == "onnx";
      } else if (key == "controller.ood_warn_z")
        c.ood_warn_z = number(val);
      else if (key == "simulation.mode") {
        require(val == "ideal" || val == "bounded" || val == "plc", "unsupported simulation mode");
        o.bounded = val != "ideal";
        o.plc = val == "plc";
      } else if (key == "simulation.plc_acceleration_m_s2")
        triple(val, o.plc_accel);
      else if (key == "simulation.plc_delay_s")
        o.plc_delay = number(val);
      else if (key == "simulation.repeat_pose_at")
        o.sim_repeat_pose_at = integer(val);
      else if (key == "simulation.encoder")
        o.sim_encoder = boolean(val);
      else if (key == "simulation.seed") {
        const double v = number(val);
        require(v >= 0 && v <= UINT32_MAX && v == std::floor(v), "seed must be uint32");
        o.seed = static_cast<std::uint32_t>(v);
      } else if (key == "simulation.noise_m")
        o.noise = number(val);
      else if (key == "simulation.latency_s")
        o.sim_latency = number(val);
      else if (key == "simulation.bias_m")
        triple(val, o.sim_bias);
      else if (key == "simulation.freeze_at")
        o.sim_freeze_at = integer(val);
      else if (key == "simulation.fail_read_at")
        o.sim_fail_read_at = integer(val);
      else if (key == "simulation.fail_send_at")
        o.sim_fail_send_at = integer(val);
      else if (key == "simulation.start_hill_m")
        triple(val, o.start_hill);
      else if (key == "simulation.fast")
        o.fast = boolean(val);
      else if (key == "simulation.open_loop")
        o.open_loop = boolean(val);
      else if (key == "simulation.max_steps")
        o.max_steps = integer(val);
      else if (key == "experiment.directory")
        o.experiment_dir = val;
      else
        throw std::invalid_argument("unknown or immutable configuration key: " + key);
    } catch (const std::exception& e) {
      throw std::invalid_argument(path + ":" + std::to_string(lineno) + ": " + e.what());
    }
  }
  o.config_files.push_back(path);
}

const pfcr::ModelSpec& model_of(const RailConfig& c) {
  const auto* model = pfcr::find_model(c.model_id);
  require(model != nullptr, "incompatible policy artifact");
  return *model;
}
void validate(const RailConfig& c) {
  model_of(c);
  require(c.period_s == pfcr::params::kDt, "PFCR artifact requires exactly 1 s control period");
  std::set<int> axes;
  for (int i = 0; i < 3; ++i) {
    require(c.rail_axis[i] >= 0 && c.rail_axis[i] < 3 && axes.insert(c.rail_axis[i]).second,
            "mapping axes must be a permutation of 0,1,2");
    require(c.sign[i] == 1 || c.sign[i] == -1, "mapping sign must be +1 or -1");
    require(std::isfinite(c.scale[i]) && c.scale[i] > 0 && std::isfinite(1.0 / c.scale[i]),
            "mapping scales must be finite and positive");
    require(std::isfinite(c.rail_min[i]) && std::isfinite(c.rail_max[i]) && c.rail_min[i] < c.rail_max[i],
            "invalid rail travel limits");
    require(std::isfinite(c.target_rail[i]) && c.target_rail[i] >= c.rail_min[i] &&
                c.target_rail[i] <= c.rail_max[i],
            "mapping target must be inside rail limits");
    require(std::isfinite(c.rail_vmax[i]) && c.rail_vmax[i] > 0 && std::isfinite(c.rail_amax[i]) &&
                c.rail_amax[i] > 0,
            "invalid motion constraints");
    require(c.thruster.scale[i] == 0.85, "actuator scale is fixed by the compatibility profile");
  }
  require(c.thruster.tau == 2.0 && c.thruster.deadzone == 0.025 && c.tracking_tol_hill_m == 1.0,
          "actuator and PFCR tracking profile are immutable");
  for (double v :
       {c.max_measurement_age_s, c.max_measurement_late_s, c.max_clock_skew_s, c.max_sample_jitter_s,
        c.communication_timeout_s, c.max_track_err_rail_m, c.max_sensor_disagreement_m, c.ood_warn_z})
    require(std::isfinite(v) && v > 0, "runtime thresholds must be finite and positive");
  require(std::isfinite(c.lead_s) && c.lead_s > 0 && c.lead_s <= 5, "lead_s must be in (0, 5] s");
  // A gain above 1/period overshoots the reference within one period.
  require(std::isfinite(c.tracking_gain_per_s) && c.tracking_gain_per_s >= 0 &&
              c.tracking_gain_per_s * c.period_s <= 1,
          "tracking_gain_per_s must be in [0, 1/period_s]");
}
void validate(const Options& o) {
  validate(o.rail);
  require(!o.lab,
          "physical hardware unavailable: drivers, feedback and independent interlocks are unverified");
#ifndef PFCR_WITH_ONNXRUNTIME
  require(!o.onnx, "ONNX requested but this build has no ONNX Runtime");
#endif
  require(std::isfinite(o.noise) && o.noise >= 0 && std::isfinite(o.sim_latency) && o.sim_latency >= 0,
          "noise and latency must be finite and nonnegative");
  require(o.max_steps >= -1 && o.sim_freeze_at >= -1 && o.sim_fail_read_at >= -1 &&
              o.sim_fail_send_at >= -1 && o.sim_repeat_pose_at >= -1,
          "step limits must be -1 or nonnegative");
  require(!o.plc || o.bounded, "PLC simulation integrates bounded motion");
  require(o.sim_trace.empty() || o.bounded, "plant trace requires bounded or PLC simulation");
  require(o.bridge_port >= 0 && o.bridge_port <= 65535, "bridge port must be 0..65535");
  require(o.bridge_port == 0 || !o.fast, "a bridged rail runs in real time; drop --fast");
  for (double a : o.plc_accel)
    require(std::isfinite(a) && a > 0, "PLC acceleration must be finite and positive");
  require(std::isfinite(o.plc_delay) && o.plc_delay >= 0 && o.plc_delay < o.rail.period_s,
          "PLC delay must be in [0, period_s)");
  const auto start = FrameMap{o.rail}.hill_to_rail(o.start_hill);
  for (int i = 0; i < 3; ++i) {
    require(std::isfinite(o.start_hill[i]) && std::isfinite(o.sim_bias[i]), "nonfinite simulation input");
    require(std::isfinite(start[i]) && start[i] >= o.rail.rail_min[i] && start[i] <= o.rail.rail_max[i],
            "start outside rail limits");
  }
}
std::string configuration_snapshot(const Options& o) {
  std::ostringstream s;
  s << std::setprecision(17) << std::boolalpha;
  auto triple_out = [&](const char* key, const auto& a) {
    s << key << " = " << a[0] << ',' << a[1] << ',' << a[2] << '\n';
  };
  const auto& c = o.rail;
  s << "[rail]\n";
  triple_out("minimum_m", c.rail_min);
  triple_out("maximum_m", c.rail_max);
  triple_out("velocity_m_s", c.rail_vmax);
  triple_out("acceleration_m_s2", c.rail_amax);
  s << "measurement_age_s = " << c.max_measurement_age_s
    << "\nmeasurement_late_s = " << c.max_measurement_late_s << "\nclock_skew_s = " << c.max_clock_skew_s
    << "\nsample_jitter_s = " << c.max_sample_jitter_s
    << "\ncommunication_timeout_s = " << c.communication_timeout_s
    << "\ntracking_error_m = " << c.max_track_err_rail_m
    << "\nsensor_disagreement_m = " << c.max_sensor_disagreement_m
    << "\ncompensate_latency = " << c.compensate_latency
    << "\ncommand_mode = " << (c.command_mode == CommandMode::Lead ? "lead" : "arrive")
    << "\nlead_s = " << c.lead_s << "\ntracking_gain_per_s = " << c.tracking_gain_per_s << "\n[mapping]\n";
  triple_out("axis", c.rail_axis);
  triple_out("sign", c.sign);
  triple_out("scale", c.scale);
  triple_out("target_m", c.target_rail);
  s << "[controller]\ntype = pfcr-dqp\nmodel_id = " << c.model_id << "\nperiod_s = " << c.period_s
    << "\nbackend = " << (o.onnx ? "onnx" : "builtin") << "\nood_warn_z = " << c.ood_warn_z
    << "\n[simulation]\nmode = " << (o.plc ? "plc" : (o.bounded ? "bounded" : "ideal"))
    << "\nseed = " << o.seed << "\nnoise_m = " << o.noise << "\nlatency_s = " << o.sim_latency
    << "\nfreeze_at = " << o.sim_freeze_at << "\nrepeat_pose_at = " << o.sim_repeat_pose_at
    << "\nfail_read_at = " << o.sim_fail_read_at << "\nfail_send_at = " << o.sim_fail_send_at
    << "\nencoder = " << o.sim_encoder << "\nplc_delay_s = " << o.plc_delay << '\n';
  triple_out("plc_acceleration_m_s2", o.plc_accel);
  triple_out("bias_m", o.sim_bias);
  triple_out("start_hill_m", o.start_hill);
  s << "fast = " << o.fast << "\nopen_loop = " << o.open_loop << "\nmax_steps = " << o.max_steps << '\n';
  return s.str();
}
}  // namespace rvd
