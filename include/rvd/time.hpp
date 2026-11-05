#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include "pfcr/pfcr_controller.hpp"

namespace rvd {
using pfcr::Vec3;
using Clock = std::chrono::steady_clock;
constexpr double kNoStamp = std::numeric_limits<double>::quiet_NaN();
inline bool has(double t) { return std::isfinite(t); }

// external clock -> now_s(): offset = min(t_received - raw_stamp)
struct ClockOffset {
  double offset = std::numeric_limits<double>::infinity();
  void update(double raw_stamp, double t_received) { offset = std::min(offset, t_received - raw_stamp); }
  bool ready() const { return std::isfinite(offset); }
  double to_framework(double raw_stamp) const { return raw_stamp + offset; }
};


}  // namespace rvd
