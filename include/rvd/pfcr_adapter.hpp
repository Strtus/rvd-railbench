#pragma once
#include "rvd/config.hpp"
#include "rvd/controller.hpp"
namespace rvd {
class PfcrAdapter final : public IController {
 public:
  explicit PfcrAdapter(const RailConfig& config);
  void initialize(const ControllerInit& settings) override;
  void reset(const StateEstimate& initial) override;
  ControllerOutput update(const Observation& observation) override;
  ControllerStatus status() const override { return status_; }
  ControllerDiagnostics diagnostics() const override;
  const pfcr::StepResult& last_result() const { return last_; }

 private:
  RailConfig config_;
  std::unique_ptr<pfcr::Controller> controller_;
  ControllerStatus status_ = ControllerStatus::Uninitialized;
  pfcr::StepResult last_;
};
}  // namespace rvd
