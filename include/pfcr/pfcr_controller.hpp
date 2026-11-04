// Observation -> policy -> CBF-QP + backup guard -> thruster model -> CWH step (dt = 1 s).
// Hill frame: x radial (docking port on +x), y along-track, z cross-track. SI units.
#pragma once

#include <array>
#include <memory>
#include <string>

#include "pfcr/pfcr_params.hpp"

namespace pfcr {

using Vec3 = std::array<double, 3>;
using Vec6 = std::array<double, 6>;
using Obs = std::array<float, params::kObsDim>;
using Action = std::array<float, 3>;

class Policy {
 public:
  virtual ~Policy() = default;
  // raw observation -> deterministic action in [-1, 1]^3
  virtual Action act(const Obs& obs) = 0;
  virtual std::string name() const = 0;
};

std::unique_ptr<Policy> make_builtin_policy(const std::string& weights_bin_path);
#ifdef PFCR_WITH_ONNXRUNTIME
std::unique_ptr<Policy> make_onnx_policy(const std::string& onnx_path);
#endif

struct ObsStats {
  std::array<double, params::kObsDim> mean{};
  std::array<double, params::kObsDim> std{};
  bool load(const std::string& weights_bin_path);
  double max_abs_z(const Obs& obs, int* index = nullptr) const;
};

struct ActuatorEmulation {
  double tau = 0.0;            // first-order lag [s]
  Vec3 scale{1.0, 1.0, 1.0};   // per-axis thrust scale
  double deadzone = 0.0;       // [N]
};

enum class Status { Run, Docked, Abort };
enum class VelocitySource { Model, Measured, FiniteDifference };
const char* status_name(Status s);

struct StepResult {
  Status status = Status::Run;
  std::string reason;
  int k = 0;
  double t = 0.0;                    // [s]
  Vec6 state{};                      // state used this step
  Obs obs{};
  Action action{};
  Vec3 u_des{}, u_act{}, u_real{};   // [N]
  Vec6 next_state{};                 // reference for t + dt
  bool backup_active = false;
  bool intervened = false;
  double dist = 0.0, speed = 0.0, min_margin = 0.0;
};

class Controller {
 public:
  Controller(std::unique_ptr<Policy> policy, ActuatorEmulation actuator = {},
             VelocitySource velocity_source = VelocitySource::Model, double tracking_tol_m = 0.10);
  ~Controller();
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;

  void reset(const Vec6& state0);
  // once per control period; vel_measured only for VelocitySource::Measured
  StepResult step(const Vec3& pos_measured, const Vec3* vel_measured = nullptr);

  bool running() const;
  const Vec6& predicted_state() const;
  const Policy& policy() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pfcr
