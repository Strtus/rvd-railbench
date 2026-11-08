#pragma once
#include <cstdio>
#include <deque>
#include <random>

#include "rvd/rail_io.hpp"
#include "rvd/runtime.hpp"
namespace rvd {
class SimulatedRailIO final : public RailIO {
 public:
  SimulatedRailIO(const Vec3& start, const Options& options);
  ~SimulatedRailIO() override;
  SimulatedRailIO(const SimulatedRailIO&) = delete;
  SimulatedRailIO& operator=(const SimulatedRailIO&) = delete;
  bool read_measurement(Measurement& measurement) override;
  bool send_target(const Command& command, CommandAck& ack) override;
  void hold() noexcept override { held_ = true; }
  double now_s() const override { return options_.fast ? virtual_time_ : RailIO::now_s(); }
  void advance_to(double time_s);  // virtual-clock mode only; never sleeps
  const Vec3& position() const { return pos_; }
  const Vec3& velocity() const { return vel_; }
  bool held() const { return held_; }

 private:
  void integrate(double now);
  void trace();
  Vec3 delayed_position(double acquired) const;
  struct Sample {
    double time;
    Vec3 position;
  };
  struct Pending {
    double time;
    Vec3 target, speed;
  };
  Options options_;
  Vec3 pos_{}, vel_{}, target_{}, speed_{};
  double virtual_time_ = 0, integrated_time_ = 0, next_trace_ = 0;
  int read_count_ = 0, send_count_ = 0;
  long long frame_ = 0;
  bool held_ = false;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  Measurement last_;
  std::deque<Sample> history_;
  std::deque<Pending> pending_;
  std::FILE* trace_ = nullptr;
};
}  // namespace rvd
