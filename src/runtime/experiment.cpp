#include "rvd/experiment.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "rvd/artifact.hpp"
#ifndef RVD_GIT_COMMIT
#define RVD_GIT_COMMIT "unknown"
#endif
#ifndef RVD_GIT_DIRTY
#define RVD_GIT_DIRTY "unknown"
#endif
namespace rvd {
namespace {
std::string quote(const std::string& value) {
  std::ostringstream s;
  s << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\')
      s << '\\' << static_cast<char>(c);
    else if (c < 32)
      s << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
    else
      s << static_cast<char>(c);
  }
  s << '"';
  return s.str();
}
std::string utc(bool filename) {
  const auto now = std::chrono::system_clock::now();
  const auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &t);
#else
  gmtime_r(&t, &tm);
#endif
  std::ostringstream s;
  s << std::put_time(&tm, filename ? "%Y%m%dT%H%M%S" : "%Y-%m-%dT%H:%M:%S");
  const auto us =
      std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count() % 1000000;
  s << (filename ? '-' : '.') << std::setw(6) << std::setfill('0') << us << 'Z';
  return s.str();
}
std::ofstream open(const std::filesystem::path& path) {
  std::ofstream out(path);
  out.exceptions(std::ios::badbit | std::ios::failbit);
  out << std::setprecision(17);
  return out;
}
}  // namespace
Experiment::Experiment(const Options& o) {
  directory_ = o.experiment_dir.empty() ? std::filesystem::path("experiments") / utc(true)
                                        : std::filesystem::path(o.experiment_dir);
  const auto export_path = std::filesystem::weakly_canonical(std::filesystem::absolute(o.log));
  const auto experiment_path = std::filesystem::weakly_canonical(std::filesystem::absolute(directory_));
  if (export_path.parent_path() == experiment_path && export_path.filename() != "trajectory.csv")
    throw std::invalid_argument(
        "--log must not overwrite structured experiment files; use trajectory.csv or an external path");
  // Atomic ownership of a new directory: never overwrite an existing experiment.
  if (!directory_.parent_path().empty()) std::filesystem::create_directories(directory_.parent_path());
  if (!std::filesystem::create_directory(directory_))
    throw std::runtime_error("experiment directory already exists: " + directory_.string());
  auto config = open(directory_ / "configuration.ini");
  config << configuration_snapshot(o);
  config.close();
  auto metadata = open(directory_ / "metadata.json");
  metadata << "{\n  \"schema_version\": 1,\n  \"experiment_id\": " << quote(directory_.filename().string())
           << ",\n  \"started_utc\": " << quote(utc(false))
           << ",\n  \"controller\": \"pfcr-dqp\",\n  \"controller_version\": \"1\",\n  \"model_id\": "
           << quote(pfcr::params::kModelId) << ",\n  \"weights_sha256\": " << quote(kWeightsSha256)
           << ",\n  \"onnx_sha256\": " << (o.onnx ? quote(kOnnxSha256) : "null")
           << ",\n  \"backend\": " << quote(o.onnx ? "onnx" : "builtin")
           << ",\n  \"git_commit_at_build\": " << quote(RVD_GIT_COMMIT)
           << ",\n  \"source_dirty_at_build\": " << quote(RVD_GIT_DIRTY) << ",\n  \"seed\": " << o.seed
           << ",\n  \"clock\": " << quote(o.fast ? "virtual-monotonic" : "steady-monotonic")
           << ",\n  \"compiler\": "
           << quote(
#ifdef _MSC_VER
                  "MSVC " + std::to_string(_MSC_VER)
#else
                  __VERSION__
#endif
                      )
           << ",\n  \"hardware\": " << (o.bridge_port > 0 ? "true" : "false")
           << ",\n  \"rail\": " << quote(o.bridge_port > 0 ? "bridge" : "simulator")
           << ",\n  \"state\": \"INITIALIZING\"\n}\n";
  metadata.close();
  inputs_ = open(directory_ / "controller.csv");
  inputs_ << "k,t_acquired,hx,hy,hz,a0,a1,a2,n0,n1,n2,n3,n4,n5,backup,intervened,status,reason\n";
  events_ = open(directory_ / "events.jsonl");
  event(0, "state", "INITIALIZING");
}
Experiment::~Experiment() {
  if (!finished_) try {
      finish("FAULT", "UNHANDLED_EXCEPTION", 2);
    } catch (...) {
    }
}
void Experiment::initial(const StateEstimate& s) {
  auto f = open(directory_ / "initial_state.csv");
  f << "x,y,z,vx,vy,vz\n";
  for (std::size_t i = 0; i < s.state.size(); ++i) f << (i ? "," : "") << s.state[i];
  f << '\n';
  f.close();
}
void Experiment::controller_step(int k, const Vec3& input, double acquired, const pfcr::StepResult& r) {
  inputs_ << k << ',' << acquired;
  for (double v : input) inputs_ << ',' << v;
  for (float v : r.action) inputs_ << ',' << v;
  for (double v : r.next_state) inputs_ << ',' << v;
  inputs_ << ',' << int(r.backup_active) << ',' << int(r.intervened) << ',' << pfcr::status_name(r.status)
          << ',' << r.reason << '\n';
  inputs_.flush();
  ++steps_;
  if (r.intervened) {
    ++interventions_;
    event(acquired, "intervention", r.backup_active ? "BACKUP" : "CBF_QP");
  }
}
void Experiment::event(double time, const std::string& type, const std::string& reason) {
  events_ << "{\"time_s\":" << (has(time) ? std::to_string(time) : "null") << ",\"type\":" << quote(type)
          << ",\"reason\":" << quote(reason) << "}\n";
  events_.flush();
}
void Experiment::finish(const std::string& outcome, const std::string& reason, int code) {
  if (finished_) return;
  inputs_ << "#complete," << steps_ << '\n';
  inputs_.close();
  event(kNoStamp, "outcome", reason);
  events_.close();
  const auto tmp = directory_ / "summary.json.tmp";
  auto f = open(tmp);
  f << "{\n  \"outcome\": " << quote(outcome) << ",\n  \"reason\": " << quote(reason)
    << ",\n  \"exit_code\": " << code << ",\n  \"controller_steps\": " << steps_
    << ",\n  \"interventions\": " << interventions_ << ",\n  \"finished_utc\": " << quote(utc(false))
    << "\n}\n";
  f.close();
  std::filesystem::rename(tmp, directory_ / "summary.json");
  auto integrity = open(directory_ / "checksums.sha256.tmp");
  for (const auto* name : {"configuration.ini", "metadata.json", "initial_state.csv", "controller.csv",
                           "events.jsonl", "trajectory.csv", "summary.json"})
    if (std::filesystem::is_regular_file(directory_ / name))
      integrity << sha256_file((directory_ / name).string()) << "  " << name << '\n';
  integrity.close();
  std::filesystem::rename(directory_ / "checksums.sha256.tmp", directory_ / "checksums.sha256");
  finished_ = true;
}
}  // namespace rvd
