// Generated from the checkpoint run configuration. Do not edit.
#pragma once

namespace pfcr::params {
inline constexpr const char* kModelId = "E_PFCR-DQP_s41_step5120000";

inline constexpr double kN = 0.0010269999999999999;    // mean motion [rad/s]
inline constexpr double kMass = 12;                     // [kg]
inline constexpr double kDt = 1;                        // control period [s]
inline constexpr double kUMax = 1;                      // per-axis force limit [N]
inline constexpr double kNu0 = 0.20000000000000001;     // [m/s]
inline constexpr double kNu1 = 2.1499999999999999 * kN; // [1/s]
inline constexpr double kMaxVel = 10;                   // [m/s]
inline constexpr double kDockRadius = 1;                // [m]
inline constexpr double kDockSpeed = 0.20000000000000001;  // [m/s]
inline constexpr double kMaxTime = 1200;                // [s]
inline constexpr double kLeaveRadius = 200;             // [m]

inline constexpr int kHistoryLen = 4;
inline constexpr double kPosScale = 100;
inline constexpr double kVelScale = 1;
inline constexpr int kObsDim = 36;

inline constexpr double kFrontBuffer = 0.25;            // [m]
inline constexpr int kGuardHorizon = 512;
inline constexpr double kGuardReleaseMargin = 0.10;
inline constexpr int kGuardMinimumDwell = 8;

inline constexpr double kTauMin = 0, kTauMax = 4;
inline constexpr double kScaleMin = 0.69999999999999996, kScaleMax = 1;
inline constexpr double kDeadzoneMin = 0, kDeadzoneMax = 0.050000000000000003;

inline constexpr double kInterveneTol = 1e-5;           // [N]
}  // namespace pfcr::params
