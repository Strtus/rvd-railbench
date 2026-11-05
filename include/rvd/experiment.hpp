#pragma once
#include "rvd/runtime.hpp"
#include "rvd/observations.hpp"
#include <filesystem>
#include <fstream>
namespace rvd {
class Experiment {
 public:
  explicit Experiment(const Options& options);
  ~Experiment();
  Experiment(const Experiment&)=delete;
  Experiment& operator=(const Experiment&)=delete;
  const std::filesystem::path& directory() const { return directory_; }
  void initial(const StateEstimate& initial);
  void controller_step(int k,const Vec3& input,double acquired,const pfcr::StepResult& result);
  void event(double time,const std::string& type,const std::string& reason);
  void finish(const std::string& outcome,const std::string& reason,int exit_code);
 private:
  std::filesystem::path directory_;
  std::ofstream inputs_,events_;
  bool finished_=false;
  int steps_=0,interventions_=0;
};
struct ReplaySummary { int steps=0; double max_error=0; };
ReplaySummary replay(const std::string& experiment_directory,const std::string& model_directory);
}  // namespace rvd
