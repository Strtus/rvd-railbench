#pragma once
#include <random>
#include "rvd/rail_io.hpp"

namespace rvd {
using pfcr::Vec3;
class SimulatedRailIO final : public RailIO {
 public:
  SimulatedRailIO(const Vec3& start, double noise_m, double latency_s, int freeze_after)
      : pos_(start), noise_(0.0, noise_m), use_noise_(noise_m > 0), latency_(latency_s), freeze_after_(freeze_after) {}
  bool read_measurement(Measurement& m) override {
    if (freeze_after_ >= 0 && frame_ > freeze_after_) { m = last_; m.t_received = now_s(); return true; }
    m.rail_pos = pos_;
    if (use_noise_) for (double& v : m.rail_pos) v += noise_(rng_);
    m.t_acquired = now_s() - latency_;
    m.sensor_raw_stamp = m.t_acquired + 1000.0;
    m.frame_id = ++frame_;
    m.t_received = now_s();
    last_ = m;
    return true;
  }
  bool send_target(const Command& c, CommandAck& ack) override {
    pos_ = c.rail_target;
    ack.t_sent = now_s();
    return true;
  }
  void hold() override {}
 private:
  Vec3 pos_;
  long long frame_ = 0;
  std::mt19937 rng_{12345};
  std::normal_distribution<double> noise_;
  bool use_noise_;
  double latency_;
  int freeze_after_;
  Measurement last_;
};


}  // namespace rvd
