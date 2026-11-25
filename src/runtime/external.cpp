#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <thread>

#include "rvd/bridge.hpp"
#include "rvd/experiment.hpp"
#include "rvd/external_peer.hpp"
#include "rvd/frames.hpp"
#include "rvd/runtime.hpp"
#include "rvd/safety.hpp"
#include "rvd/simulation.hpp"

namespace rvd {
namespace {
enum class Reply { Invalid, Reference, Stop };

struct Step {
  Measurement measurement;
  Vec3 hill{kNoStamp, kNoStamp, kNoStamp};
  Vec3 target{kNoStamp, kNoStamp, kNoStamp};
  Vec3 velocity{kNoStamp, kNoStamp, kNoStamp};
  Command command;
  CommandAck acknowledgment;
  double started = kNoStamp, received = kNoStamp, tracking_error = kNoStamp;

  Step(int k, double tick, double period) {
    measurement.rail_pos.fill(kNoStamp);
    command.k = k;
    command.t_tick = tick;
    command.t_arrive = tick + period;
    command.rail_target.fill(kNoStamp);
    command.rail_velocity.fill(kNoStamp);
  }
};

std::string state_line(const Step& step) {
  std::ostringstream state;
  state.imbue(std::locale::classic());
  state << std::setprecision(17) << "STATE " << step.command.k << ' ' << step.measurement.t_acquired << ' '
        << step.measurement.frame_id;
  for (double v : step.hill) state << ' ' << v;
  return state.str();
}

Reply parse_reply(const std::string& line, Step& step) {
  std::istringstream fields(line);
  fields.imbue(std::locale::classic());
  std::string verb;
  int seq = -1;
  if (!(fields >> verb >> seq) || seq != step.command.k) return Reply::Invalid;
  if (verb == "STOP") return (fields >> std::ws).eof() ? Reply::Stop : Reply::Invalid;
  if (verb != "REF") return Reply::Invalid;
  Vec3 target{}, velocity{};
  for (double& v : target)
    if (!(fields >> v) || !has(v)) return Reply::Invalid;
  for (double& v : velocity)
    if (!(fields >> v) || !has(v)) return Reply::Invalid;
  if (!(fields >> std::ws).eof()) return Reply::Invalid;
  step.target = target;
  step.velocity = velocity;
  return Reply::Reference;
}

class HoldOnExit {
 public:
  explicit HoldOnExit(RailIO& io) : io_(io) {}
  ~HoldOnExit() { hold(); }
  void hold() noexcept {
    if (held_) return;
    held_ = true;
    io_.hold();
  }

 private:
  RailIO& io_;
  bool held_ = false;
};

void write_step(std::ofstream& csv, const Step& step, const char* status, const std::string& reason) {
  const auto& m = step.measurement;
  const auto& c = step.command;
  auto value = [&](double v) {
    csv << ',';
    if (has(v)) csv << v;
  };
  csv << c.k;
  for (double v : {c.t_tick, m.t_acquired, m.t_received}) value(v);
  csv << ',';
  if (m.frame_id >= 0) csv << m.frame_id;
  value(c.t_tick - m.t_acquired);
  for (double v : m.rail_pos) value(v);
  for (double v : m.encoder_pos) value(v);
  value(m.t_encoder);
  value(step.tracking_error);
  for (const auto* vector :
       {&step.hill, &step.target, &step.velocity, &c.rail_target, &c.rail_velocity, &c.rail_reference})
    for (double v : *vector) value(v);
  for (double v : {step.started, step.received, step.acknowledgment.t_sent, step.acknowledgment.t_plc_ack})
    value(v);
  csv << ',' << status << ',' << reason << '\n';
  csv.flush();
}
}  // namespace

int run_external(const Options& opt) {
  validate(opt);
  const auto& config = opt.rail;
  const FrameMap frames(config);
  SafetySupervisor safety(config);
  Experiment experiment(opt);
  ExternalPeer controller(opt.external_port);
  std::unique_ptr<RailIO> io;
  if (opt.bridge_port > 0) {
    io = std::make_unique<BridgeRailIO>(opt.bridge_port, config.communication_timeout_s);
  } else {
    io = std::make_unique<SimulatedRailIO>(frames.hill_to_rail(opt.start_hill), opt);
  }
  HoldOnExit hold(*io);

  const auto path = experiment.directory() / "trajectory.csv";
  std::ofstream csv(path);
  csv.exceptions(std::ios::failbit | std::ios::badbit);
  csv.imbue(std::locale::classic());
  csv << std::setprecision(17)
      << "k,t_tick,t_acquired,t_received,frame_id,meas_age_s,rail_x,rail_y,rail_z,enc_x,enc_y,enc_z,"
         "t_encoder,track_err_m,hx,hy,hz,nx,ny,nz,nvx,nvy,nvz,tx,ty,tz,tvx,tvy,tvz,rx,ry,rz,"
         "t_ctrl_start,t_ctrl_end,t_sent,t_plc_ack,status,reason\n";

  const double start = io->now_s();
  Measurement previous;
  Vec3 last_target{}, last_velocity{};
  std::string outcome = "COMPLETE", reason = "MAX_STEPS";
  int exit_code = 0;
  for (int k = 0;; ++k) {
    const double tick = start + k * config.period_s;
    std::this_thread::sleep_until(io->at(tick));
    Step step(k, tick, config.period_s);
    auto& m = step.measurement;
    auto& command = step.command;
    const auto* tracking_target = k > 0 ? &last_target : nullptr;
    const auto* previous_sample = k > 0 ? &previous : nullptr;
    if (io->stop_requested()) {
      safety.fault("STOP_REQUESTED");
    } else if (safety.communication(io->read_measurement(m), false)) {
      const double sampled = io->now_s();
      if (has(m.t_acquired) && m.frame_id >= 0 &&
          safety.measurement(m, previous_sample, tick, sampled, tracking_target, io->stop_requested())) {
        step.hill = frames.rail_to_hill(m.rail_pos);
        if (!std::all_of(step.hill.begin(), step.hill.end(), has)) safety.fault("INVALID_HILL_MEASUREMENT");
      } else if (safety.state() != RuntimeState::Fault) {
        safety.fault("INVALID_EXTERNAL_MEASUREMENT");
      }
      if (tracking_target && std::all_of(m.rail_pos.begin(), m.rail_pos.end(), has)) {
        const auto& p = m.rail_pos;
        step.tracking_error = std::hypot(p[0] - last_target[0], p[1] - last_target[1], p[2] - last_target[2]);
      }
    }
    if (k == 0 && safety.state() != RuntimeState::Fault) {
      safety.transition(RuntimeState::Ready);
      experiment.event(io->now_s(), "state", "READY");
      safety.transition(RuntimeState::Running);
      experiment.event(io->now_s(), "state", "RUNNING");
    }

    bool stop = false;
    if (safety.state() == RuntimeState::Running && k != opt.max_steps) {
      const auto due = io->at(command.t_arrive);
      step.started = io->now_s();
      std::string reply;
      if (!controller.exchange(state_line(step), reply, due)) {
        switch (controller.failure()) {
          case ExternalPeer::Failure::Timeout:
            safety.fault("EXTERNAL_TIMEOUT");
            break;
          case ExternalPeer::Failure::Protocol:
            safety.fault("EXTERNAL_PROTOCOL");
            break;
          default:
            safety.fault("EXTERNAL_DISCONNECTED");
            break;
        }
      } else if (safety.deadline(io->now_s(), command.t_arrive)) {
        step.received = io->now_s();
        const auto kind = parse_reply(reply, step);
        if (kind == Reply::Stop) {
          stop = true;
        } else if (kind != Reply::Reference) {
          safety.fault("EXTERNAL_PROTOCOL");
        } else {
          command.rail_target = frames.hill_to_rail(step.target);
          command.rail_reference = command.rail_target;
          command.rail_velocity = frames.hill_velocity_to_rail(step.velocity);
          if (safety.command(command, last_velocity)) {
            // Positioning drives determine direction from target minus measured position, not the speed sign.
            for (int axis = 0; axis < 3; ++axis) {
              const double delta = command.rail_target[axis] - m.rail_pos[axis];
              if (std::fabs(delta) > 1e-9 &&
                  (command.rail_velocity[axis] == 0 || (delta < 0) != (command.rail_velocity[axis] < 0))) {
                safety.fault("RAIL_VELOCITY_DIRECTION");
                break;
              }
            }
          }
          if (safety.state() == RuntimeState::Running && safety.deadline(io->now_s(), command.t_arrive)) {
            if (safety.communication(io->send_target(command, step.acknowledgment), true) &&
                safety.acknowledgment(step.acknowledgment, io->now_s(), command.t_arrive,
                                      opt.bridge_port > 0)) {
              if (io->stop_requested()) {
                safety.fault("STOP_REQUESTED");
              } else {
                experiment.external_step(k, step.hill, m.t_acquired, m.frame_id, step.target, step.velocity);
                last_target = command.rail_target;
                last_velocity = command.rail_velocity;
                previous = m;
              }
            }
          }
        }
      }
    }

    if (safety.state() == RuntimeState::Fault) {
      reason = safety.reason();
      outcome = "FAULT";
      exit_code = k == 0 && !has(step.started) ? 3 : 1;
      hold.hold();
      experiment.event(io->now_s(), "fault", reason);
      write_step(csv, step, "ABORT", reason);
      break;
    }
    if (stop || k == opt.max_steps) {
      if (stop) {
        outcome = "STOPPED";
        reason = "CLIENT_STOP";
      }
      hold.hold();
      write_step(csv, step, outcome.c_str(), reason);
      break;
    }
    write_step(csv, step, "RUN", "");
  }

  safety.transition(RuntimeState::Stopped);
  experiment.event(io->now_s(), "state", "STOPPED");
  csv.close();
  if (!opt.log.empty() &&
      std::filesystem::weakly_canonical(opt.log) != std::filesystem::weakly_canonical(path))
    std::filesystem::copy_file(path, opt.log, std::filesystem::copy_options::overwrite_existing);
  experiment.finish(outcome, reason, exit_code);
  controller.notify("END " + outcome + ' ' + reason);
  std::printf("external %s: %s (%s)\n", outcome.c_str(), reason.c_str(),
              experiment.directory().string().c_str());
  return exit_code;
}
}  // namespace rvd
