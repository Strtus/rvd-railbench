#include "rvd/runtime.hpp"
#include "rvd/frames.hpp"
#include "rvd/pfcr_adapter.hpp"
#include "rvd/rail_io.hpp"
#include "rvd/simulation.hpp"
#include "rvd/telemetry.hpp"
#include "rvd/safety.hpp"
#include "rvd/experiment.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <thread>
namespace rvd {
using namespace pfcr;
int run(const Options& opt) {
  const auto& cfg = opt.rail;
  SafetySupervisor safety(cfg);
  FrameMap fm{cfg};
  validate(opt);
  PfcrAdapter ctrl(cfg);
  ctrl.initialize({opt.model_dir, opt.onnx});
  ObsStats stats;
  if (!stats.load(opt.model_dir + "/pfcr_policy_weights.bin")) throw std::runtime_error("cannot load stats");

  Experiment experiment(opt);
  std::unique_ptr<RailIO> io;
  if (opt.lab) throw std::runtime_error("physical hardware is unavailable: drivers and interlocks are not implemented");
  auto simulation = std::make_unique<SimulatedRailIO>(fm.hill_to_rail(opt.start_hill), opt);
  auto* sim = simulation.get();
  io = std::move(simulation);
  struct HoldOnExit { RailIO& io; ~HoldOnExit() { io.hold(); } } hold_guard{*io};

  const auto trajectory_path = experiment.directory() / "trajectory.csv";
  std::unique_ptr<std::FILE, decltype(&std::fclose)> log_owner(std::fopen(trajectory_path.string().c_str(), "w"), &std::fclose);
  std::FILE* log = log_owner.get();
  if (!log) { std::fprintf(stderr, "cannot open log %s\n", opt.log.c_str()); return 2; }
  write_csv_header(log);

  Measurement m0;
  if (!safety.communication(io->read_measurement(m0), false) ||
      !safety.measurement(m0, nullptr, io->now_s(), io->now_s(), nullptr, io->stop_requested())) {
    experiment.event(io->now_s(), "fault", safety.reason());
    experiment.finish("FAULT", safety.reason(), 3);
    std::fprintf(stderr, "initial measurement rejected: %s\n", safety.reason().c_str()); return 3;
  }
  safety.transition(RuntimeState::Ready);
  experiment.event(io->now_s(), "state", "READY");
  safety.transition(RuntimeState::Running);
  experiment.event(io->now_s(), "state", "RUNNING");
  const Vec3 h0 = fm.rail_to_hill(m0.rail_pos);
  const StateEstimate initial{Frame::Hill, {h0[0], h0[1], h0[2], 0, 0, 0}, m0.t_acquired};
  ctrl.reset(initial);
  experiment.initial(initial);
  std::printf("model %s, policy %s, start hill (%.3f, %.3f, %.3f) m\n", params::kModelId, ctrl.diagnostics().backend.c_str(),
              h0[0], h0[1], h0[2]);

  const double t_start = io->now_s();
  int ood_warnings = 0, exit_code = 0;
  std::string final_outcome, final_reason;
  Vec3 last_target = m0.rail_pos, last_vel{0, 0, 0};
  double prev_acq = kNoStamp;
  Measurement previous = m0;
  for (int k = 0;; ++k) {
    const double t_tick = t_start + k * cfg.period_s;
    if (opt.fast) sim->advance_to(t_tick);
    if (!opt.fast) {
      std::this_thread::sleep_until(io->at(t_tick));
      while (io->now_s() < t_tick) std::this_thread::yield();
    }
    const double t_wake = io->now_s();
    std::string reason;
    bool ok = true;
    if (io->stop_requested()) { ok = safety.fault("STOP_REQUESTED"); reason = safety.reason(); }
    else if (opt.max_steps >= 0 && k >= opt.max_steps) { ok = false; reason = "MAX_STEPS"; }

    Measurement m;
    m.rail_pos = last_target;
    if (ok && !safety.communication(io->read_measurement(m), false)) { ok = false; reason = safety.reason(); m.rail_pos = last_target; }
    const double t_after_read = io->now_s();

    const double t_ref = has(m.t_acquired) ? m.t_acquired : (has(m.t_received) ? m.t_received : t_wake);
    const double age = t_tick - t_ref;
    const double dt_acq = has(m.t_acquired) && has(prev_acq) ? m.t_acquired - prev_acq : kNoStamp;
    if (ok && !safety.measurement(m, k > 0 ? &previous : nullptr, t_tick, t_after_read, nullptr)) {
      ok = false; reason = safety.reason();
    }
    if (ok && has(dt_acq) && std::fabs(dt_acq - cfg.period_s) > cfg.max_sample_jitter_s)
      std::printf("k=%d irregular sampling: %.3f s between acquisitions\n", k, dt_acq);
    Vec3 meas = m.rail_pos;
    double comp_shift = 0.0;
    if (cfg.compensate_latency && k > 0 && has(m.t_acquired)) {
      const double dt = std::clamp(t_tick - m.t_acquired, -cfg.max_measurement_late_s, cfg.max_measurement_age_s);
      for (int i = 0; i < 3; ++i) meas[i] += last_vel[i] * dt;
      comp_shift = std::sqrt((meas[0] - m.rail_pos[0]) * (meas[0] - m.rail_pos[0]) + (meas[1] - m.rail_pos[1]) *
                             (meas[1] - m.rail_pos[1]) + (meas[2] - m.rail_pos[2]) * (meas[2] - m.rail_pos[2]));
    }
    const double track_err = k == 0 ? 0.0
        : std::sqrt((meas[0] - last_target[0]) * (meas[0] - last_target[0]) + (meas[1] - last_target[1]) *
                    (meas[1] - last_target[1]) + (meas[2] - last_target[2]) * (meas[2] - last_target[2]));
    if (ok) {
      Measurement compensated = m; compensated.rail_pos = meas;
      if (!safety.measurement(compensated, nullptr, t_tick, t_after_read, k > 0 ? &last_target : nullptr)) {
        ok = false; reason = safety.reason();
      }
    }
    const Vec3 feed = (opt.open_loop && k > 0) ? last_target : meas;

    const double t_ctrl_start = io->now_s();
    StepResult r;
    if (ok) {
      const auto h = fm.rail_to_hill(feed);
      ctrl.update({StateEstimate{Frame::Hill, {h[0], h[1], h[2], 0, 0, 0}, m.t_acquired}, {}});
      r = ctrl.last_result();
      experiment.controller_step(k, h, m.t_acquired, r);
      if (!safety.controller_output(r)) { ok = false; reason = safety.reason(); }
      else if (r.status != Status::Run) {
        ok = false; reason = r.reason;
        if (r.status == Status::Abort) safety.fault(reason);
      }
    } else {
      const Vec3 h = fm.rail_to_hill(feed);
      r.status = Status::Abort;
      r.k = k;
      r.state = {h[0], h[1], h[2], 0.0, 0.0, 0.0};
      r.next_state = r.state;
      r.dist = std::sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
    }
    const double t_ctrl_end = io->now_s();

    Command cmd;
    cmd.k = k;
    cmd.t_tick = t_tick;
    cmd.t_arrive = t_tick + cfg.period_s;
    cmd.rail_target = fm.hill_to_rail({r.next_state[0], r.next_state[1], r.next_state[2]});
    for (int i = 0; ok && i < 3; ++i)
      cmd.rail_velocity[i] = (cmd.rail_target[i] - feed[i]) / cfg.period_s;
    if (ok && (!safety.command(cmd, last_vel) || !safety.deadline(t_ctrl_end, cmd.t_arrive))) {
      ok = false; reason = safety.reason();
    }
    int zi = 0;
    const double z = r.status == Status::Run ? stats.max_abs_z(r.obs, &zi) : 0.0;
    if (z > cfg.ood_warn_z && ++ood_warnings <= 20)
      std::printf("k=%d observation %d is %.1f sigma from training\n", k, zi, z);

    CommandAck ack;
    if (ok && (!safety.communication(io->send_target(cmd, ack), true) ||
               !safety.acknowledgment(ack, io->now_s(), cmd.t_arrive, false))) {
      ok = false; reason = safety.reason();
    }
    if (!ok) io->hold();

    const char* status = ok ? "RUN" : (r.status == Status::Docked ? "DOCKED" : "ABORT");
    write_csv_row(log, k, t_tick, t_wake, m, age, dt_acq, comp_shift, track_err, r, cmd,
                  t_ctrl_start, t_ctrl_end, ack, z, zi, status, reason);

    if (k % 10 == 0 || !ok) {
      char age_txt[32] = "-";
      if (has(age)) std::snprintf(age_txt, sizeof age_txt, "%.0f ms", age * 1e3);
      std::printf("k=%4d hill (%7.3f %6.3f %6.3f) |v| %.3f  u_act (%5.2f %5.2f %5.2f) N  age %s  %s %s\n", k,
                  r.state[0], r.state[1], r.state[2], r.speed, r.u_act[0], r.u_act[1], r.u_act[2], age_txt, status,
                  reason.c_str());
    }
    if (!ok) {
      exit_code = r.status == Status::Docked && safety.state() != RuntimeState::Fault ? 0 : 1;
      if (safety.state() == RuntimeState::Fault) experiment.event(io->now_s(), "fault", reason);
      if (safety.state() == RuntimeState::Running) safety.transition(RuntimeState::Hold);
      experiment.event(io->now_s(), "state", state_name(safety.state()));
      final_outcome = exit_code == 0 ? "DOCKED" : (reason == "MAX_STEPS" ? "LIMIT_REACHED" : "FAULT");
      final_reason = reason;
      safety.transition(RuntimeState::Stopped);
      experiment.event(io->now_s(), "state", "STOPPED");
      std::printf("%s: %s at k=%d, distance %.3f m, speed %.3f m/s\n", status, reason.c_str(), k, r.dist, r.speed);
      break;
    }
    if (!opt.fast && io->now_s() > t_tick + 0.5 * cfg.period_s) std::printf("k=%d overran half a period\n", k);
    last_target = cmd.rail_target;
    last_vel = cmd.rail_velocity;
    prev_acq = m.t_acquired;
    previous = m;
  }
  if (std::ferror(log)) throw std::runtime_error("telemetry write failed");
  if (std::fclose(log_owner.release()) != 0) throw std::runtime_error("telemetry close failed");
  // Retain the original --log/default CSV path as a compatibility export.
  if (std::filesystem::absolute(opt.log) != std::filesystem::absolute(trajectory_path)) {
    std::ifstream input(trajectory_path, std::ios::binary);
    std::ofstream output(opt.log, std::ios::binary | std::ios::trunc);
    output << input.rdbuf();
    if (!input || !output) throw std::runtime_error("cannot export compatibility CSV: " + opt.log);
  }
  experiment.finish(final_outcome, final_reason, exit_code);
  std::printf("experiment %s; CSV %s (%d observation warnings)\n", experiment.directory().string().c_str(), opt.log.c_str(), ood_warnings);
  return exit_code;
}


}  // namespace rvd
