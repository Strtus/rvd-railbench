#pragma once
#include "rvd/config.hpp"

namespace rvd {
using pfcr::Vec3;
struct FrameMap {
  explicit FrameMap(const RailConfig& config) : c(config) { validate(c); }
  const RailConfig c;
  Vec3 rail_to_hill(const Vec3& rail) const {
    Vec3 h{};
    for (int j = 0; j < 3; ++j)
      h[j] = c.sign[j] * (rail[c.rail_axis[j]] - c.target_rail[c.rail_axis[j]]) / c.scale[j];
    return h;
  }
  Vec3 hill_to_rail(const Vec3& hill) const {
    Vec3 r{};
    for (int j = 0; j < 3; ++j)
      r[c.rail_axis[j]] = c.target_rail[c.rail_axis[j]] + c.sign[j] * c.scale[j] * hill[j];
    return r;
  }
};

}  // namespace rvd
