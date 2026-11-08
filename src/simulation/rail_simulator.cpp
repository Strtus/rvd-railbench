#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "rvd/simulation.hpp"
namespace rvd {
SimulatedRailIO::SimulatedRailIO(const Vec3& start, const Options& o)
    : options_(o), pos_(start), target_(start), rng_(o.seed), noise_(0, o.noise > 0 ? o.noise : 1) {
  validate(o);
  for (double v : start)
    if (!has(v)) throw std::invalid_argument("nonfinite simulator start");
  history_.push_back({0, start});
  if (!o.sim_trace.empty()) {
    if (!o.bounded) throw std::invalid_argument("plant trace requires bounded or PLC simulation");
    trace_ = std::fopen(o.sim_trace.c_str(), "w");
    if (!trace_) throw std::runtime_error("cannot open plant trace: " + o.sim_trace);
    std::fputs("t,x,y,z,vx,vy,vz,target_x,target_y,target_z,speed_x,speed_y,speed_z\n", trace_);
    trace();
  }
}
SimulatedRailIO::~SimulatedRailIO() {
  if (trace_) std::fclose(trace_);
}
void SimulatedRailIO::advance_to(double t) {
  if (!options_.fast || !has(t) || t < virtual_time_)
    throw std::invalid_argument("invalid virtual clock advance");
  integrate(t);
  virtual_time_ = t;
}
void SimulatedRailIO::trace() {
  std::fprintf(trace_, "%.4f", integrated_time_);
  for (const Vec3* v : {&pos_, &vel_, &target_, &speed_})
    for (double x : *v) std::fprintf(trace_, ",%.7f", x);
  std::fputc('\n', trace_);
  next_trace_ += .01;
}
void SimulatedRailIO::integrate(double now) {
  if (!options_.bounded) {
    integrated_time_ = now;
    return;
  }
  // Fixed maximum integration step; acceleration and speed are bounded at every substep.
  while (integrated_time_ < now) {
    const double h = std::min(.001, now - integrated_time_);
    while (!pending_.empty() && pending_.front().time <= integrated_time_) {
      target_ = pending_.front().target;
      speed_ = pending_.front().speed;
      pending_.pop_front();
    }
    for (int i = 0; i < 3; ++i) {
      const double error = target_[i] - pos_[i];
      const double a = options_.plc ? options_.plc_accel[i] : options_.rail.rail_amax[i];
      if (options_.plc && std::fabs(error) <= 1e-6 && std::fabs(vel_[i]) <= a * h) {
        pos_[i] = target_[i];  // positioning complete: the drive holds the target
        vel_[i] = 0;
        continue;
      }
      // A PLC positioning move cruises at the commanded speed and decelerates to rest at its target.
      const double cruise =
          options_.plc ? std::min(speed_[i], options_.rail.rail_vmax[i]) : options_.rail.rail_vmax[i];
      const double desired =
          held_ ? 0 : std::copysign(std::min(cruise, std::sqrt(2 * a * std::fabs(error))), error);
      const double next = std::clamp(desired, vel_[i] - a * h, vel_[i] + a * h);
      pos_[i] += .5 * (vel_[i] + next) * h;
      vel_[i] = next;
    }
    integrated_time_ += h;
    history_.push_back({integrated_time_, pos_});
    while (history_.size() > 2 && history_[1].time < integrated_time_ - options_.sim_latency - 1)
      history_.pop_front();
    if (trace_ && integrated_time_ >= next_trace_ - 1e-9) trace();
  }
}
Vec3 SimulatedRailIO::delayed_position(double acquired) const {
  if (!options_.bounded) return pos_;  // preserve original teleport sensor semantics
  if (acquired <= history_.front().time) return history_.front().position;
  for (std::size_t i = 1; i < history_.size(); ++i)
    if (history_[i].time >= acquired) {
      const auto& a = history_[i - 1];
      const auto& b = history_[i];
      const double f = (acquired - a.time) / (b.time - a.time);
      Vec3 result{};
      for (int j = 0; j < 3; ++j) result[j] = a.position[j] + f * (b.position[j] - a.position[j]);
      return result;
    }
  return history_.back().position;
}
bool SimulatedRailIO::read_measurement(Measurement& m) {
  if (options_.sim_fail_read_at >= 0 && read_count_ >= options_.sim_fail_read_at) return false;
  ++read_count_;
  const double now = now_s();
  integrate(now);
  if (options_.sim_freeze_at >= 0 && frame_ > options_.sim_freeze_at) {
    m = last_;
    m.t_received = now;
    if (options_.sim_encoder) {
      m.encoder_pos = pos_;
      m.t_encoder = now;
    }
    return true;
  }
  const bool repeat = options_.sim_repeat_pose_at >= 0 && frame_ > options_.sim_repeat_pose_at;
  const Vec3 previous = last_.rail_pos;
  m = {};
  m.t_acquired = now - options_.sim_latency;
  m.t_received = now;
  m.rail_pos = delayed_position(m.t_acquired);
  for (int i = 0; i < 3; ++i) {
    m.rail_pos[i] += options_.sim_bias[i];
    if (options_.noise > 0) m.rail_pos[i] += noise_(rng_);
  }
  if (repeat) m.rail_pos = previous;  // tracker lost the body but keeps publishing its last pose
  if (options_.sim_encoder) {
    m.encoder_pos = pos_;
    m.t_encoder = now;
  }
  m.sensor_raw_stamp = m.t_acquired + 1000;
  m.frame_id = ++frame_;
  m.synchronization = Synchronization::Synchronized;
  m.timestamp_uncertainty_s = 0;
  last_ = m;
  return true;
}
bool SimulatedRailIO::send_target(const Command& c, CommandAck& ack) {
  if (held_ || (options_.sim_fail_send_at >= 0 && send_count_ >= options_.sim_fail_send_at)) return false;
  if (!has(c.t_arrive) || !has(c.t_tick) || c.t_arrive <= now_s()) return false;
  Vec3 reference = c.rail_target, speed{};
  for (int i = 0; i < 3; ++i) {
    if (has(c.rail_reference[i])) reference[i] = c.rail_reference[i];
    if (!has(c.rail_target[i]) || !has(c.rail_velocity[i]) || c.rail_target[i] < options_.rail.rail_min[i] ||
        c.rail_target[i] > options_.rail.rail_max[i] || reference[i] < options_.rail.rail_min[i] ||
        reference[i] > options_.rail.rail_max[i] ||
        std::fabs(c.rail_velocity[i]) > options_.rail.rail_vmax[i])
      return false;
    speed[i] = std::fabs(c.rail_velocity[i]);
  }
  ++send_count_;
  const double now = now_s();
  integrate(now);
  if (options_.plc) {
    pending_.push_back({now + options_.plc_delay, c.rail_target, speed});
  } else {
    target_ = reference;
    speed_ = speed;
    if (!options_.bounded) pos_ = target_;
  }
  ack = {};
  ack.t_sent = now;
  return true;  // no fictional PLC acknowledgment
}
}  // namespace rvd
