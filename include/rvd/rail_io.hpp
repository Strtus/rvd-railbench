#pragma once
#include "pfcr/pfcr_controller.hpp"
#include "rvd/time.hpp"

namespace rvd {
using pfcr::Vec3;
// all t_* fields are seconds on RailIO::now_s(); NaN = not provided
struct Measurement {
  Vec3 rail_pos{};  // [m]
  double t_acquired = kNoStamp;
  double t_received = kNoStamp;
  double t_user1 = kNoStamp;
  double t_user2 = kNoStamp;
  double sensor_raw_stamp = kNoStamp;  // sensor clock, logged only
  Synchronization synchronization = Synchronization::Unknown;
  double timestamp_uncertainty_s = kNoStamp;
  long long frame_id = -1;
};

struct Command {
  int k = 0;
  Vec3 rail_target{};    // [m], reach at t_arrive
  Vec3 rail_velocity{};  // [m/s], constant over the period
  double t_tick = 0.0;
  double t_arrive = 0.0;
};

struct CommandAck {
  double t_sent = kNoStamp;
  double t_plc_ack = kNoStamp;
  double t_user3 = kNoStamp;
  double t_user4 = kNoStamp;
  double plc_raw_stamp = kNoStamp;  // PLC clock, logged only
};

class RailIO {
 public:
  virtual ~RailIO() = default;
  virtual bool read_measurement(Measurement& m) = 0;
  virtual bool send_target(const Command& c, CommandAck& ack) = 0;
  virtual void hold() noexcept = 0;
  virtual bool stop_requested() { return false; }
  virtual double now_s() const { return std::chrono::duration<double>(Clock::now() - t0_).count(); }
  Clock::time_point at(double t_s) const {
    return t0_ + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(t_s));
  }

 protected:
  Clock::time_point t0_ = Clock::now();
};

}  // namespace rvd
