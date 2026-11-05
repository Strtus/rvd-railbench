#include "rvd/runtime.hpp"
#include "rvd/frames.hpp"
#include "rvd/rail_io.hpp"
#include "rvd/simulation.hpp"
#include "rvd/telemetry.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <thread>
namespace rvd {
using namespace pfcr;
int run(const Options& opt, const RailConfig& cfg) {
  FrameMap fm{cfg};
  std::unique_ptr<Policy> policy;
#ifdef PFCR_WITH_ONNXRUNTIME
  if (opt.onnx) policy = make_onnx_policy(opt.model_dir + "/pfcr_policy.onnx");
#endif
  if (!policy) policy = make_builtin_policy(opt.model_dir + "/pfcr_policy_weights.bin");
  ObsStats stats;
  if (!stats.load(opt.model_dir + "/pfcr_policy_weights.bin")) { std::fprintf(stderr, "cannot load stats\n"); return 2; }
  Controller ctrl(std::move(policy), cfg.thruster, VelocitySource::Model, cfg.tracking_tol_hill_m);

  std::unique_ptr<RailIO> io;
  if (opt.lab) throw std::runtime_error("physical hardware is unavailable: drivers and interlocks are not implemented");
  io = std::make_unique<SimulatedRailIO>(fm.hill_to_rail(opt.start_hill), opt.noise, opt.sim_latency,
                                              opt.sim_freeze_at);

  std::FILE* log = std::fopen(opt.log.c_str(), "w");
  if (!log) { std::fprintf(stderr, "cannot open log %s\n", opt.log.c_str()); return 2; }
  write_csv_header(log);

  Measurement m0;
  if (!io->read_measurement(m0)) { std::fprintf(stderr, "initial read_measurement failed\n"); io->hold(); return 3; }
  const Vec3 h0 = fm.rail_to_hill(m0.rail_pos);
  ctrl.reset({h0[0], h0[1], h0[2], 0.0, 0.0, 0.0});
  std::printf("model %s, policy %s, start hill (%.3f, %.3f, %.3f) m\n", params::kModelId, ctrl.policy().name().c_str(),
              h0[0], h0[1], h0[2]);

  const double t_start = io->now_s();
  int ood_warnings = 0, exit_code = 0;
  Vec3 last_target = m0.rail_pos, last_vel{0, 0, 0};
  double prev_acq = kNoStamp;
  long long prev_frame = m0.frame_id;
  for (int k = 0;; ++k) {
    const double t_tick = t_start + k * cfg.period_s;
    if (!opt.fast) {
      std::this_thread::sleep_until(io->at(t_tick));
      while (io->now_s() < t_tick) std::this_thread::yield();
    }
    const double t_wake = io->now_s();
    std::string reason;
    bool ok = true;
    if (io->stop_requested()) { ok = false; reason = "STOP_REQUESTED"; }
    else if (opt.max_steps >= 0 && k >= opt.max_steps) { ok = false; reason = "MAX_STEPS"; }

    Measurement m;
    m.rail_pos = last_target;
    if (ok && !io->read_measurement(m)) { ok = false; reason = "READ_FAILED"; m.rail_pos = last_target; }
    const double t_after_read = io->now_s();

    const double t_ref = has(m.t_acquired) ? m.t_acquired : (has(m.t_received) ? m.t_received : t_wake);
    const double age = opt.fast ? kNoStamp : t_tick - t_ref;
    const double dt_acq = has(m.t_acquired) && has(prev_acq) ? m.t_acquired - prev_acq : kNoStamp;
    if (ok && !opt.fast) {
      if (k > 0 && m.frame_id >= 0 && m.frame_id == prev_frame) { ok = false; reason = "SENSOR_FROZEN"; }
      else if (k > 0 && has(m.t_acquired) && m.t_acquired == prev_acq) { ok = false; reason = "SENSOR_FROZEN"; }
      else if (t_ref > t_after_read + cfg.max_clock_skew_s) { ok = false; reason = "STAMP_IN_FUTURE"; }
      else if (age > cfg.max_measurement_age_s) { ok = false; reason = "STALE_MEASUREMENT"; }
      else if (-age > cfg.max_measurement_late_s) { ok = false; reason = "MEASUREMENT_LATE"; }
      if (ok && has(dt_acq) && std::fabs(dt_acq - cfg.period_s) > cfg.max_sample_jitter_s)
        std::printf("k=%d irregular sampling: %.3f s between acquisitions\n", k, dt_acq);
    }
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
    if (ok && track_err > cfg.max_track_err_rail_m) { ok = false; reason = "RAIL_TRACKING_ERROR"; }
    const Vec3 feed = (opt.open_loop && k > 0) ? last_target : meas;

    const double t_ctrl_start = io->now_s();
    StepResult r;
    if (ok) {
      r = ctrl.step(fm.rail_to_hill(feed));
      if (r.status != Status::Run) { ok = false; reason = r.reason; }
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
    for (int i = 0; ok && i < 3; ++i) {
      cmd.rail_velocity[i] = (cmd.rail_target[i] - feed[i]) / cfg.period_s;
      if (cmd.rail_target[i] < cfg.rail_min[i] || cmd.rail_target[i] > cfg.rail_max[i]) { ok = false; reason = "RAIL_ENVELOPE"; }
      else if (std::fabs(cmd.rail_velocity[i]) > cfg.rail_vmax[i]) { ok = false; reason = "RAIL_SPEED"; }
    }
    int zi = 0;
    const double z = r.status == Status::Run ? stats.max_abs_z(r.obs, &zi) : 0.0;
    if (z > cfg.ood_warn_z && ++ood_warnings <= 20)
      std::printf("k=%d observation %d is %.1f sigma from training\n", k, zi, z);

    CommandAck ack;
    if (ok && !io->send_target(cmd, ack)) { ok = false; reason = "SEND_FAILED"; }
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
      exit_code = r.status == Status::Docked ? 0 : 1;
      std::printf("%s: %s at k=%d, distance %.3f m, speed %.3f m/s\n", status, reason.c_str(), k, r.dist, r.speed);
      break;
    }
    if (!opt.fast && io->now_s() > t_tick + 0.5 * cfg.period_s) std::printf("k=%d overran half a period\n", k);
    last_target = cmd.rail_target;
    last_vel = cmd.rail_velocity;
    prev_acq = m.t_acquired;
    prev_frame = m.frame_id;
  }
  std::fclose(log);
  std::printf("log written to %s (%d observation warnings)\n", opt.log.c_str(), ood_warnings);
  return exit_code;
}


}  // namespace rvd
