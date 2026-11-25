#pragma once
#include <filesystem>
#include <fstream>

#include "rvd/observations.hpp"
#include "rvd/runtime.hpp"
namespace rvd {
inline constexpr char kExternalControllerColumns[] =
    "k,t_acquired,frame_id,hx,hy,hz,ref_hx,ref_hy,ref_hz,ref_hvx,ref_hvy,ref_hvz";

class Experiment {
 public:
  explicit Experiment(const Options& options);
  ~Experiment();
  Experiment(const Experiment&) = delete;
  Experiment& operator=(const Experiment&) = delete;
  const std::filesystem::path& directory() const { return directory_; }
  void initial(const StateEstimate& initial);
  void controller_step(int k, const Vec3& input, double acquired, const pfcr::StepResult& result);
  void external_step(int k, const Vec3& input, double acquired, long long frame_id, const Vec3& target_hill,
                     const Vec3& velocity_hill);
  void event(double time, const std::string& type, const std::string& reason);
  void finish(const std::string& outcome, const std::string& reason, int exit_code);

 private:
  std::filesystem::path directory_;
  std::ofstream inputs_, events_;
  bool external_, finished_ = false;
  int steps_ = 0, interventions_ = 0;
};
struct ReplaySummary {
  int steps = 0;
  double max_error = 0;
};
// An empty model directory selects the recorded model under policies/.
ReplaySummary replay(const std::string& experiment_directory, const std::string& model_directory);
}  // namespace rvd
