// Checkpoints the controller is ported for. Constants come from each run configuration. Do not edit.
#pragma once

#include <string>

namespace pfcr {

enum class Docking {
  CaptureSphere,  // |r| <= kDockRadius and |v| <= kDockSpeed
  ContactPlane    // judged where the chaser first crosses x = contact_x
};

struct ModelSpec {
  const char* id;
  int obs_dim;
  // line-of-sight cone (HOCBF row and observed margin); apex at -cone_apex_offset on the docking axis
  bool los_cone;
  double cone_half_angle;   // [rad]
  double cone_apex_offset;  // [m]
  bool bearing;             // observed (y, z) / max(x, kBearingXMin)
  // u_des = s * u_max * action, s falling from 1 at x >= action_scale_far to action_scale_min at contact_x
  double action_scale_min;  // 1 disables the scaling
  double action_scale_far;  // [m]
  Docking docking;
  double contact_x;                  // [m]
  double contact_lateral_max;        // [m]
  double contact_lateral_speed_max;  // [m/s]
  double contact_closing_min, contact_closing_max;  // [m/s]
  const char* weights_sha256;
  const char* onnx_sha256;
};

inline constexpr double kConeA1 = 0.5, kConeA2 = 0.5;
inline constexpr double kConeMarginScale = 100.0 * 100.0;  // [m^2]
inline constexpr double kBearingXMin = 0.5;                // [m]

// IDSS IDD Rev E contact conditions (translational terms).
inline constexpr ModelSpec kModels[] = {
    {"IDSS_PFCR-DQP_s49_step10240000", 40, true, 0.17453292519943295, 0.70140000000000002, true, 0.25, 10,
     Docking::ContactPlane, 1, 0.10000000000000001, 0.040000000000000001, 0.050000000000000003,
     0.10000000000000001, "36b5f8d1037b6c659e2826a5c26905219181e967cf8e98837fe8a8e26c25eb30",
     "8eeb0f94aa94a2cb5cd132619453bb8bb35111f9fb6618ddf469a259b77a60ea"},
    {"E_PFCR-DQP_s41_step5120000", 36, false, 0, 0, false, 1, 10, Docking::CaptureSphere, 1, 0, 0, 0, 0,
     "599c666e99f8bcbd95dc234f023f1d919717fedd652659444976d4d2bfaefdee",
     "9b39f7c79521f58d9f724b82d692e0a4b910bd70891fcd1135c4f9415e23f2ee"},
};

inline const ModelSpec& default_model() { return kModels[0]; }
// nullptr for an unknown id
inline const ModelSpec* find_model(const std::string& id) {
  for (const auto& m : kModels)
    if (id == m.id) return &m;
  return nullptr;
}

}  // namespace pfcr
