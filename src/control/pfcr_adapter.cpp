#include "rvd/pfcr_adapter.hpp"

#include <cmath>
#include <stdexcept>

#include "rvd/artifact.hpp"
namespace rvd {
namespace {
void check_estimate(const StateEstimate& s) {
  if (s.frame != Frame::Hill) throw std::invalid_argument("PFCR requires Hill frame state");
  for (double v : s.state)
    if (!std::isfinite(v)) throw std::invalid_argument("nonfinite controller input");
}
}  // namespace
PfcrAdapter::PfcrAdapter(const RailConfig& c) : config_(c) { validate(config_); }
void PfcrAdapter::initialize(const ControllerInit& s) {
  // Failed initialization must not leave a previously initialized controller usable.
  controller_.reset();
  status_ = ControllerStatus::Uninitialized;
  verify_artifact(s.artifact_directory, s.onnx);
  std::unique_ptr<pfcr::Policy> policy;
  if (s.onnx) {
#ifdef PFCR_WITH_ONNXRUNTIME
    policy = pfcr::make_onnx_policy(s.artifact_directory + "/pfcr_policy.onnx");
#else
    throw std::invalid_argument("ONNX Runtime unavailable");
#endif
  } else
    policy = pfcr::make_builtin_policy(s.artifact_directory + "/pfcr_policy_weights.bin");
  controller_ = std::make_unique<pfcr::Controller>(std::move(policy), config_.thruster,
                                                   pfcr::VelocitySource::Model, config_.tracking_tol_hill_m);
  status_ = ControllerStatus::Ready;
}
void PfcrAdapter::reset(const StateEstimate& s) {
  if (!controller_) throw std::logic_error("controller not initialized");
  check_estimate(s);
  controller_->reset(s.state);
  last_ = {};
  status_ = ControllerStatus::Running;
}
ControllerOutput PfcrAdapter::update(const Observation& o) {
  if (!controller_ || status_ != ControllerStatus::Running)
    throw std::logic_error("controller must be reset before update");
  if (!o.estimate)
    throw std::invalid_argument("PFCR requires a state estimate; raw-only observations are unsupported");
  check_estimate(*o.estimate);
  last_ = controller_->step({o.estimate->state[0], o.estimate->state[1], o.estimate->state[2]});
  status_ =
      last_.status == pfcr::Status::Run
          ? ControllerStatus::Running
          : (last_.status == pfcr::Status::Docked ? ControllerStatus::Completed : ControllerStatus::Fault);
  return {status_, StateEstimate{Frame::Hill, last_.next_state, o.estimate->acquired_s + config_.period_s},
          last_.reason};
}
ControllerDiagnostics PfcrAdapter::diagnostics() const {
  return {"pfcr-dqp",
          "1",
          controller_ ? controller_->policy().name() : "uninitialized",
          last_.intervened,
          last_.backup_active,
          last_.min_margin};
}
}  // namespace rvd
