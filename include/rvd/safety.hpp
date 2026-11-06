#pragma once
#include "rvd/config.hpp"
#include "rvd/hardware.hpp"
#include "rvd/rail_io.hpp"
namespace rvd {
enum class RuntimeState { Initializing, Ready, Running, Hold, Fault, Stopped };
const char* state_name(RuntimeState state);
std::string validate_timestamps(const Measurement& m, const Measurement* previous, double tick, double now,
                                const RailConfig& c);
class SafetySupervisor {
 public:
  explicit SafetySupervisor(const RailConfig& c) : config_(c) { validate(c); }
  RuntimeState state() const { return state_; }
  const std::string& reason() const { return reason_; }
  void transition(RuntimeState next);
  bool fault(const std::string& reason);
  bool measurement(const Measurement& m, const Measurement* previous, double tick, double now,
                   const Vec3* last_target, bool emergency_stop = false);
  bool command(const Command& command, const Vec3& previous_velocity);
  bool controller_output(const pfcr::StepResult& result);
  bool communication(bool success, bool sending);
  bool acknowledgment(const CommandAck& ack, double now, double deadline, bool require_plc);
  bool deadline(double completed, double deadline);
  bool hardware(const HardwareSafetyStatus& status, double now);

 private:
  RailConfig config_;
  RuntimeState state_ = RuntimeState::Initializing;
  std::string reason_;
};
}  // namespace rvd
