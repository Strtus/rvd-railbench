#include "rvd/safety.hpp"

#include <cmath>
#include <stdexcept>
namespace rvd {
const char* state_name(RuntimeState s) {
  switch (s) {
    case RuntimeState::Initializing:
      return "INITIALIZING";
    case RuntimeState::Ready:
      return "READY";
    case RuntimeState::Running:
      return "RUNNING";
    case RuntimeState::Hold:
      return "HOLD";
    case RuntimeState::Fault:
      return "FAULT";
    case RuntimeState::Stopped:
      return "STOPPED";
  }
  return "UNKNOWN";
}
std::string validate_timestamps(const Measurement& m, const Measurement* previous, double tick, double now,
                                const RailConfig& c) {
  if (!has(tick) || !has(now)) return "INVALID_RUNTIME_CLOCK";
  if (!has(m.t_acquired) && !has(m.t_received)) return "MISSING_TIMESTAMP";
  if (m.synchronization == Synchronization::Unknown) return "UNSYNCHRONIZED_TIMESTAMP";
  if (!has(m.timestamp_uncertainty_s) || m.timestamp_uncertainty_s < 0 ||
      m.timestamp_uncertainty_s > c.max_clock_skew_s)
    return "TIMESTAMP_UNCERTAINTY";
  if (previous) {
    if ((m.frame_id >= 0 && previous->frame_id >= 0 && m.frame_id < previous->frame_id) ||
        (has(m.t_acquired) && has(previous->t_acquired) && m.t_acquired < previous->t_acquired))
      return "OUT_OF_ORDER_MEASUREMENT";
    if ((m.frame_id >= 0 && m.frame_id == previous->frame_id) ||
        (has(m.t_acquired) && m.t_acquired == previous->t_acquired))
      return "SENSOR_FROZEN";
  }
  const double ref = has(m.t_acquired) ? m.t_acquired : m.t_received;
  if (ref > now + c.max_clock_skew_s || (has(m.t_received) && m.t_received > now + c.max_clock_skew_s) ||
      (has(m.t_acquired) && has(m.t_received) && m.t_acquired > m.t_received + c.max_clock_skew_s))
    return "STAMP_IN_FUTURE";
  if (tick - ref > c.max_measurement_age_s) return "STALE_MEASUREMENT";
  if (ref - tick > c.max_measurement_late_s) return "MEASUREMENT_LATE";
  if (!has(m.t_received) || now - m.t_received > c.communication_timeout_s) return "COMMUNICATION_TIMEOUT";
  return {};
}
void SafetySupervisor::transition(RuntimeState next) {
  const bool valid =
      (state_ == RuntimeState::Initializing && next == RuntimeState::Ready) ||
      (state_ == RuntimeState::Ready && (next == RuntimeState::Running || next == RuntimeState::Stopped)) ||
      (state_ == RuntimeState::Running && (next == RuntimeState::Hold || next == RuntimeState::Stopped)) ||
      (state_ == RuntimeState::Hold && (next == RuntimeState::Ready || next == RuntimeState::Stopped)) ||
      (state_ == RuntimeState::Fault && next == RuntimeState::Stopped);
  if (!valid) throw std::logic_error("invalid runtime state transition");
  state_ = next;
}
bool SafetySupervisor::fault(const std::string& reason) {
  if (state_ != RuntimeState::Fault && state_ != RuntimeState::Stopped) {
    state_ = RuntimeState::Fault;
    reason_ = reason;
  }
  return false;
}
bool SafetySupervisor::measurement(const Measurement& m, const Measurement* previous, double tick, double now,
                                   const Vec3* target, bool emergency_stop) {
  if (state_ == RuntimeState::Fault || state_ == RuntimeState::Stopped) return false;
  if (emergency_stop) return fault("STOP_REQUESTED");
  for (int i = 0; i < 3; ++i) {
    if (!has(m.rail_pos[i])) return fault("INVALID_MEASUREMENT");
    if (m.rail_pos[i] < config_.rail_min[i] || m.rail_pos[i] > config_.rail_max[i])
      return fault("MEASURED_RAIL_ENVELOPE");
  }
  const auto stamp = validate_timestamps(m, previous, tick, now, config_);
  if (!stamp.empty()) return fault(stamp);
  if (has(m.encoder_pos[0]) && has(m.encoder_pos[1]) && has(m.encoder_pos[2])) {
    double gap = 0, moved = 0;
    if (previous && m.rail_pos == previous->rail_pos && has(previous->encoder_pos[0]) &&
        has(previous->encoder_pos[1]) && has(previous->encoder_pos[2])) {
      for (int i = 0; i < 3; ++i)
        moved +=
            (m.encoder_pos[i] - previous->encoder_pos[i]) * (m.encoder_pos[i] - previous->encoder_pos[i]);
      // A live tracker never reports a bit-identical pose while the axes move.
      if (std::sqrt(moved) > 5e-4) return fault("MEASUREMENT_REPEATED");
    }
    for (int i = 0; i < 3; ++i)
      gap += (m.rail_pos[i] - m.encoder_pos[i]) * (m.rail_pos[i] - m.encoder_pos[i]);
    if (std::sqrt(gap) > config_.max_sensor_disagreement_m) return fault("SENSOR_DISAGREEMENT");
  }
  if (target) {
    double err = 0;
    for (int i = 0; i < 3; ++i) {
      if (!has((*target)[i])) return fault("INVALID_TRACKING_REFERENCE");
      err += (m.rail_pos[i] - (*target)[i]) * (m.rail_pos[i] - (*target)[i]);
    }
    if (std::sqrt(err) > config_.max_track_err_rail_m) return fault("RAIL_TRACKING_ERROR");
  }
  return true;
}
bool SafetySupervisor::command(const Command& c, const Vec3& previous_velocity) {
  if (state_ != RuntimeState::Running) return fault("COMMAND_OUTSIDE_RUNNING");
  if (!has(c.t_tick) || !has(c.t_arrive) || std::fabs((c.t_arrive - c.t_tick) - config_.period_s) > 1e-9)
    return fault("INVALID_COMMAND_TIMING");
  for (int i = 0; i < 3; ++i) {
    if (!has(c.rail_target[i]) || !has(c.rail_velocity[i]) || !has(previous_velocity[i]))
      return fault("INVALID_COMMAND");
    if (c.rail_target[i] < config_.rail_min[i] || c.rail_target[i] > config_.rail_max[i] ||
        (has(c.rail_reference[i]) &&
         (c.rail_reference[i] < config_.rail_min[i] || c.rail_reference[i] > config_.rail_max[i])))
      return fault("RAIL_ENVELOPE");
    if (std::fabs(c.rail_velocity[i]) > config_.rail_vmax[i]) return fault("RAIL_SPEED");
    if (std::fabs(c.rail_velocity[i] - previous_velocity[i]) / config_.period_s > config_.rail_amax[i])
      return fault("RAIL_ACCELERATION");
  }
  return true;
}
bool SafetySupervisor::controller_output(const pfcr::StepResult& r) {
  auto finite = [](const auto& a) {
    for (auto v : a)
      if (!has(v)) return false;
    return true;
  };
  if (!finite(r.state) || !finite(r.next_state) || !finite(r.action) || !finite(r.obs) || !finite(r.u_des) ||
      !finite(r.u_act) || !finite(r.u_real) || !has(r.dist) || !has(r.speed) || !has(r.min_margin))
    return fault("INVALID_CONTROLLER_OUTPUT");
  return state_ != RuntimeState::Fault && state_ != RuntimeState::Stopped;
}
bool SafetySupervisor::communication(bool success, bool sending) {
  if (!success) return fault(sending ? "SEND_FAILED" : "READ_FAILED");
  return state_ != RuntimeState::Fault && state_ != RuntimeState::Stopped;
}
bool SafetySupervisor::acknowledgment(const CommandAck& ack, double now, double deadline, bool require_plc) {
  if (!has(ack.t_sent) || !has(now) || !has(deadline) || ack.t_sent > now + config_.max_clock_skew_s ||
      now - ack.t_sent > config_.communication_timeout_s || now > deadline)
    return fault("COMMAND_TIMEOUT");
  if (require_plc && (!has(ack.t_plc_ack) || ack.t_plc_ack < ack.t_sent - config_.max_clock_skew_s ||
                      ack.t_plc_ack > now + config_.max_clock_skew_s ||
                      ack.t_plc_ack - ack.t_sent > config_.communication_timeout_s))
    return fault("PLC_ACK_TIMEOUT");
  return state_ != RuntimeState::Fault && state_ != RuntimeState::Stopped;
}
bool SafetySupervisor::deadline(double completed, double due) {
  if (deadline_missed(completed, due)) return fault("CONTROL_OVERRUN");
  return state_ != RuntimeState::Fault && state_ != RuntimeState::Stopped;
}
bool SafetySupervisor::hardware(const HardwareSafetyStatus& s, double now) {
  if (!s.valid || !has(s.acquired_s) || !has(now) || s.acquired_s > now + config_.max_clock_skew_s ||
      now - s.acquired_s > config_.communication_timeout_s)
    return fault("HARDWARE_STATUS_UNAVAILABLE");
  if (s.emergency_stop) return fault("STOP_REQUESTED");
  if (!s.interlocks_ready || !s.drives_ready) return fault("HARDWARE_NOT_READY");
  return state_ != RuntimeState::Fault && state_ != RuntimeState::Stopped;
}
}  // namespace rvd
