#pragma once
#include "pfcr/pfcr_controller.hpp"

namespace rvd {
using pfcr::Vec3;
struct RailConfig {
  double period_s = 1.0;

  // rail[rail_axis[j]] = target_rail[rail_axis[j]] + sign[j] * scale[j] * hill[j]
  int rail_axis[3] = {0, 1, 2};
  double sign[3] = {+1.0, +1.0, +1.0};
  double scale[3] = {1.0, 0.1, 0.1};
  double target_rail[3] = {0.0, 0.0, 0.0};    // TODO docking port in rail coordinates [m]

  double rail_min[3] = {0.0, -0.5, -0.5};     // [m]
  double rail_max[3] = {20.0, 0.5, 0.5};      // [m]
  double rail_vmax[3] = {0.5, 0.2, 0.2};      // [m/s]

  double max_measurement_age_s = 0.2;
  double max_measurement_late_s = 0.3;
  double max_clock_skew_s = 0.005;
  double max_sample_jitter_s = 0.2;
  bool compensate_latency = false;

  double max_track_err_rail_m = 0.05;
  double tracking_tol_hill_m = 1.0;
  double ood_warn_z = 6.5;
  pfcr::ActuatorEmulation thruster{2.0, {0.85, 0.85, 0.85}, 0.025};
};


}  // namespace rvd
