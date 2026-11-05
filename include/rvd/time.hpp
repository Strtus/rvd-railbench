#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace rvd {
using Clock = std::chrono::steady_clock;
constexpr double kNoStamp = std::numeric_limits<double>::quiet_NaN();
inline bool has(double t) { return std::isfinite(t); }
enum class Synchronization { Unknown, Estimated, Synchronized };
// Min(receipt - device) is an offset estimate, not proof of clock synchronization.
struct ClockOffset {
  double offset=std::numeric_limits<double>::infinity();
  double uncertainty_s=std::numeric_limits<double>::infinity();
  Synchronization status=Synchronization::Unknown;
  void update(double raw, double received, double uncertainty) {
    if(!has(raw)||!has(received)||!has(uncertainty)||uncertainty<0)
      throw std::invalid_argument("invalid clock calibration sample");
    offset=std::min(offset,received-raw);uncertainty_s=uncertainty;
    status=Synchronization::Estimated;
  }
  bool ready() const { return has(offset); }
  double to_framework(double raw) const {
    if(!ready()||!has(raw))throw std::logic_error("clock mapping unavailable");
    return raw+offset;
  }
};
inline bool deadline_missed(double completed, double deadline) {
  return !has(completed)||!has(deadline)||completed>deadline;
}
}  // namespace rvd
