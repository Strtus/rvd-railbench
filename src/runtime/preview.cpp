#include "rvd/preview.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "rvd/artifact.hpp"
#include "rvd/runtime.hpp"

namespace rvd {
namespace {
namespace P = pfcr::params;
using pfcr::Vec6;
using Row = std::map<std::string, std::string>;

std::vector<std::string> split(std::string line) {
  if (!line.empty() && line.back() == '\r') line.pop_back();
  std::vector<std::string> out;
  std::stringstream stream(line);
  std::string cell;
  while (std::getline(stream, cell, ',')) out.push_back(cell);
  if (!line.empty() && line.back() == ',') out.emplace_back();
  return out;
}
std::vector<Row> read_rows(const std::string& path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot read trajectory");
  std::string line;
  std::getline(stream, line);
  const auto names = split(line);
  std::vector<Row> rows;
  while (std::getline(stream, line)) {
    const auto values = split(line);
    if (values.size() != names.size()) throw std::runtime_error("incomplete trajectory row");
    Row row;
    for (size_t i = 0; i < names.size(); ++i) row[names[i]] = values[i];
    rows.push_back(std::move(row));
  }
  if (stream.bad() || rows.empty()) throw std::runtime_error("empty or unreadable trajectory");
  return rows;
}
double number(const Row& row, const std::string& name) {
  const auto& text = row.at(name);
  size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) throw std::runtime_error("invalid preview input");
  return value;
}
Vec6 state(const Row& row) {
  return {number(row, "hx"),  number(row, "hy"),  number(row, "hz"),
          number(row, "hvx"), number(row, "hvy"), number(row, "hvz")};
}
Vec3 force(const Row& row, const std::string& prefix) {
  return {number(row, prefix + "0"), number(row, prefix + "1"), number(row, prefix + "2")};
}
double norm(const Vec6& s, int offset) {
  return std::sqrt(s[offset] * s[offset] + s[offset + 1] * s[offset + 1] + s[offset + 2] * s[offset + 2]);
}
// Matches the deployed CWH Euler step and actuator update. Kept outside the controller:
// a counterfactual must never become a selectable hardware control mode.
Vec3 actuate(const Vec3& command, const Vec3& previous, const pfcr::ActuatorEmulation& p) {
  Vec3 result = command;
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(result[i]) < p.deadzone) result[i] = 0;
    result[i] *= p.scale[i];
    if (p.tau > 1e-9) result[i] = previous[i] + P::kDt / (p.tau + P::kDt) * (result[i] - previous[i]);
    result[i] = std::clamp(result[i], -P::kUMax, P::kUMax);
  }
  return result;
}
Vec6 cwh_step(const Vec6& s, const Vec3& force) {
  const double n = P::kN;
  Vec6 drift{s[3], s[4], s[5], 3.0 * (n * n) * s[0] + 2.0 * n * s[4], -2.0 * n * s[3], -(n * n) * s[2]};
  Vec6 next{};
  for (int i = 0; i < 6; ++i) {
    next[i] = s[i] + (drift[i] + (i < 3 ? 0.0 : (1.0 / P::kMass) * force[i - 3])) * P::kDt;
    if (!std::isfinite(next[i])) throw std::runtime_error("nonfinite preview state");
  }
  return next;
}
void position(std::ostream& stream, const Vec6& s) {
  for (int i = 0; i < 3; ++i) stream << ',' << s[i];
}
void push(std::array<float, 3 * P::kHistoryLen>& history, const Vec3& value) {
  for (int i = 0; i < 3 * (P::kHistoryLen - 1); ++i) history[i] = history[i + 3];
  for (int i = 0; i < 3; ++i) history[3 * (P::kHistoryLen - 1) + i] = static_cast<float>(value[i] / P::kUMax);
}
pfcr::Obs observation(const Vec6& s, const std::array<float, 12>& nominal,
                      const std::array<float, 12>& applied) {
  pfcr::Obs o{};
  for (int i = 0; i < 3; ++i) {
    o[i] = static_cast<float>(s[i] / P::kPosScale);
    o[i + 3] = static_cast<float>(s[i + 3] / P::kVelScale);
  }
  for (int i = 0; i < 12; ++i) {
    o[i + 6] = nominal[i];
    o[i + 18] = applied[i];
  }
  o[30] = static_cast<float>(P::kNu0 + P::kNu1 * norm(s, 0) - norm(s, 3));
  for (int i = 0; i < 3; ++i)
    o[i + 31] =
        static_cast<float>((P::kMaxVel * P::kMaxVel - s[i + 3] * s[i + 3]) / (P::kMaxVel * P::kMaxVel));
  // There are no filter interventions in this independent rollout (o[34:36] = 0).
  return o;
}
}  // namespace

void write_preview(const std::string& experiment, const std::string& model, const std::string& output) {
  Options options;
  load_config(experiment + "/configuration.ini", options);
  validate(options);
  verify_artifact(model, options.onnx);
  const auto rows = read_rows(experiment + "/trajectory.csv");
  if (number(rows.front(), "k") != 0 || rows.front().at("status") != "RUN")
    throw std::runtime_error("preview needs a run starting at k=0");
  // Refuse an existing directory, so a recording or an earlier comparison cannot be overwritten.
  if (!std::filesystem::create_directory(output)) throw std::runtime_error("preview output already exists");
  std::unique_ptr<pfcr::Policy> policy;
#ifdef PFCR_WITH_ONNXRUNTIME
  if (options.onnx) policy = pfcr::make_onnx_policy(model + "/pfcr_policy.onnx");
#else
  if (options.onnx) throw std::runtime_error("ONNX preview unavailable in this build");
#endif
  if (!policy) policy = pfcr::make_builtin_policy(model + "/pfcr_policy_weights.bin");
  std::ofstream nominal(output + "/nominal.csv"), local(output + "/local.csv");
  nominal.exceptions(std::ios::badbit | std::ios::failbit);
  local.exceptions(std::ios::badbit | std::ios::failbit);
  nominal << std::setprecision(17) << "k,hx,hy,hz,hvx,hvy,hvz,ud0,ud1,ud2,status\n";
  Vec6 s = state(rows.front());
  Vec3 previous{};
  std::array<float, 12> nominal_history{}, applied_history{};
  for (int k = 0; k <= static_cast<int>(P::kMaxTime / P::kDt); ++k) {
    std::string status = "RUN";
    if (norm(s, 0) <= P::kDockRadius && norm(s, 3) <= P::kDockSpeed && s[0] >= P::kFrontBuffer - 1e-8)
      status = "DOCKED";
    else if (s[0] < P::kFrontBuffer - 1e-8)
      status = "FRONT_PLANE_CROSSED";
    else if (norm(s, 0) > P::kLeaveRadius)
      status = "LEFT_MODEL_DOMAIN";
    else if (k * P::kDt >= P::kMaxTime)
      status = "TIMEOUT";
    Vec3 desired{};
    if (status == "RUN") {
      const auto action = policy->act(observation(s, nominal_history, applied_history));
      for (int i = 0; i < 3; ++i)
        desired[i] = std::clamp(static_cast<double>(action[i]) * P::kUMax, -P::kUMax, P::kUMax);
    }
    nominal << k;
    for (double value : s) nominal << ',' << value;
    for (double value : desired) nominal << ',' << value;
    nominal << ',' << status << '\n';
    if (status != "RUN") break;
    previous = actuate(desired, previous, options.rail.thruster);
    s = cwh_step(s, previous);
    push(nominal_history, desired);
    push(applied_history, previous);
  }
  local << std::setprecision(17) << "event_k,dt,raw_x,raw_y,raw_z,filtered_x,filtered_y,filtered_z\n";
  for (size_t i = 0; i < rows.size(); ++i) {
    const auto& row = rows[i];
    if (row.at("status") != "RUN" || row.at("intervened") != "1") continue;
    if (number(row, "k") != static_cast<double>(i)) throw std::runtime_error("nonconsecutive preview inputs");
    Vec6 raw = state(row), filtered = raw;
    Vec3 raw_force = i ? force(rows[i - 1], "ur") : Vec3{};
    Vec3 filtered_force = raw_force;
    const auto desired = force(row, "ud"), corrected = force(row, "ua");
    // Both branches hold the current command for 8 s. Neither re-evaluates the policy/filter.
    for (int step = 0; step <= 8; ++step) {
      local << row.at("k") << ',' << step * P::kDt;
      position(local, raw);
      position(local, filtered);
      local << '\n';
      raw_force = actuate(desired, raw_force, options.rail.thruster);
      filtered_force = actuate(corrected, filtered_force, options.rail.thruster);
      raw = cwh_step(raw, raw_force);
      filtered = cwh_step(filtered, filtered_force);
    }
  }
}
}  // namespace rvd
