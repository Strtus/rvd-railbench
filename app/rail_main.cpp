// 1 s control loop: measurement -> Controller::step -> rail target.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <thread>

#include "pfcr/pfcr_controller.hpp"

#if defined(_WIN32) && defined(RVD_WINDOWS_TIMER)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <timeapi.h>
#ifdef _MSC_VER
#pragma comment(lib, "winmm.lib")
#endif
#endif

using namespace pfcr;
using Clock = std::chrono::steady_clock;

constexpr double kNoStamp = std::numeric_limits<double>::quiet_NaN();
inline bool has(double t) { return std::isfinite(t); }

struct RailConfig {
  double period_s = 1.0;

  // rail[rail_axis[j]] = target_rail[rail_axis[j]] + sign[j] * scale[j] * hill[j]
  int rail_axis[3] = {0, 1, 2};
  double sign[3] = {+1.0, +1.0, +1.0};
  double scale[3] = {1.0, 0.1, 0.1};
  double target_rail[3] = {0.0, 0.0, 0.0};    // TODO docking port in rail coordinates [m]

  double rail_min[3] = {0.0, -0.5, -0.5};     // [m]
  double rail_max[3] = {20.0, 0.5, 0.5};      // [m]
  double rail_vmax[3] = {0.5, 0.2, 0.2};      // [m/s]

  double max_measurement_age_s = 0.2;
  double max_measurement_late_s = 0.3;
  double max_clock_skew_s = 0.005;
  double max_sample_jitter_s = 0.2;
  bool compensate_latency = false;

  double max_track_err_rail_m = 0.05;
  double tracking_tol_hill_m = 1.0;
  double ood_warn_z = 6.5;
  ActuatorEmulation thruster{2.0, {0.85, 0.85, 0.85}, 0.025};
};

struct FrameMap {
  const RailConfig& c;
  Vec3 rail_to_hill(const Vec3& rail) const {
    Vec3 h{};
    for (int j = 0; j < 3; ++j) h[j] = c.sign[j] * (rail[c.rail_axis[j]] - c.target_rail[c.rail_axis[j]]) / c.scale[j];
    return h;
  }
  Vec3 hill_to_rail(const Vec3& hill) const {
    Vec3 r{};
    for (int j = 0; j < 3; ++j) r[c.rail_axis[j]] = c.target_rail[c.rail_axis[j]] + c.sign[j] * c.scale[j] * hill[j];
    return r;
  }
};

// all t_* fields are seconds on RailIO::now_s(); NaN = not provided
struct Measurement {
  Vec3 rail_pos{};                      // [m]
  double t_acquired = kNoStamp;
  double t_received = kNoStamp;
  double t_user1 = kNoStamp;
  double t_user2 = kNoStamp;
  double sensor_raw_stamp = kNoStamp;   // sensor clock, logged only
  long long frame_id = -1;
};

struct Command {
  int k = 0;
  Vec3 rail_target{};                   // [m], reach at t_arrive
  Vec3 rail_velocity{};                 // [m/s], constant over the period
  double t_tick = 0.0;
  double t_arrive = 0.0;
};

struct CommandAck {
  double t_sent = kNoStamp;
  double t_plc_ack = kNoStamp;
  double t_user3 = kNoStamp;
  double t_user4 = kNoStamp;
  double plc_raw_stamp = kNoStamp;      // PLC clock, logged only
};

// external clock -> now_s(): offset = min(t_received - raw_stamp)
struct ClockOffset {
  double offset = std::numeric_limits<double>::infinity();
  void update(double raw_stamp, double t_received) { offset = std::min(offset, t_received - raw_stamp); }
  bool ready() const { return std::isfinite(offset); }
  double to_framework(double raw_stamp) const { return raw_stamp + offset; }
};

class RailIO {
 public:
  virtual ~RailIO() = default;
  virtual bool read_measurement(Measurement& m) = 0;
  virtual bool send_target(const Command& c, CommandAck& ack) = 0;
  virtual void hold() = 0;
  virtual bool stop_requested() { return false; }
  double now_s() const { return std::chrono::duration<double>(Clock::now() - t0_).count(); }
  Clock::time_point at(double t_s) const {
    return t0_ + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(t_s));
  }
 protected:
  Clock::time_point t0_ = Clock::now();
};

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

class LabRailIO final : public RailIO {
 public:
  bool read_measurement(Measurement& m) override {
    // TODO m.rail_pos; m.t_acquired (clock.to_framework(raw) or now_s()); m.sensor_raw_stamp; m.frame_id;
    //      m.t_received = now_s()
    (void)m;
    return false;
  }
  bool send_target(const Command& c, CommandAck& ack) override {
    // TODO move to c.rail_target by c.t_arrive at c.rail_velocity; ack.t_sent / ack.t_plc_ack / ack.plc_raw_stamp
    (void)c; (void)ack;
    return false;
  }
  void hold() override {
    // TODO stop all axes
  }
  bool stop_requested() override {
    // TODO emergency stop
    return false;
  }
 private:
  ClockOffset clock;
};

std::string find_model_dir() {
  const std::string rel = "model/" + std::string(params::kModelId);
  for (const char* up : {"", "../", "../../", "../../../"}) {
    const std::string d = std::string(up) + rel;
    if (std::FILE* f = std::fopen((d + "/pfcr_policy_weights.bin").c_str(), "rb")) { std::fclose(f); return d; }
  }
  return rel;
}

struct Options {
  std::string model_dir = find_model_dir();
  bool onnx = false, lab = false, fast = false;
  bool open_loop = false;
  int max_steps = -1;
  double noise = 0.0005;
  double sim_latency = 0.010;
  int sim_freeze_at = -1;
  Vec3 start_hill{18.0, 0.3, -0.2};
  std::string log = "pfcr_rail_log.csv";
};

static void put(std::FILE* f, double v) {
  if (has(v)) std::fprintf(f, "%.6f,", v); else std::fputs(",", f);
}

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
  if (opt.lab) io = std::make_unique<LabRailIO>();
  else io = std::make_unique<SimulatedRailIO>(fm.hill_to_rail(opt.start_hill), opt.noise, opt.sim_latency,
                                              opt.sim_freeze_at);

  std::FILE* log = std::fopen(opt.log.c_str(), "w");
  if (!log) { std::fprintf(stderr, "cannot open log %s\n", opt.log.c_str()); return 2; }
  std::fprintf(log,
               "k,t_tick,t_wake,t_acquired,t_received,t_user1,t_user2,sensor_raw_stamp,frame_id,meas_age_s,dt_acq_s,"
               "rail_x,rail_y,rail_z,comp_shift_m,track_err_m,hx,hy,hz,hvx,hvy,hvz,a0,a1,a2,ud0,ud1,ud2,ua0,ua1,ua2,"
               "ur0,ur1,ur2,nx,ny,nz,nvx,nvy,nvz,tx,ty,tz,tvx,tvy,tvz,t_ctrl_start,t_ctrl_end,t_sent,t_plc_ack,"
               "t_user3,t_user4,plc_raw_stamp,backup,intervened,obs_max_z,obs_max_z_idx,status,reason\n");

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
    std::fprintf(log, "%d,", k);
    for (double v : {t_tick, t_wake, m.t_acquired, m.t_received, m.t_user1, m.t_user2, m.sensor_raw_stamp}) put(log, v);
    std::fprintf(log, "%lld,", m.frame_id);
    for (double v : {age, dt_acq}) put(log, v);
    for (double v : m.rail_pos) std::fprintf(log, "%.6f,", v);
    for (double v : {comp_shift, track_err}) std::fprintf(log, "%.6f,", v);
    for (double v : r.state) std::fprintf(log, "%.6f,", v);
    for (float v : r.action) std::fprintf(log, "%.6f,", v);
    for (const Vec3* u : {&r.u_des, &r.u_act, &r.u_real}) for (double v : *u) std::fprintf(log, "%.6f,", v);
    for (double v : r.next_state) std::fprintf(log, "%.6f,", v);
    for (double v : cmd.rail_target) std::fprintf(log, "%.6f,", v);
    for (double v : cmd.rail_velocity) std::fprintf(log, "%.6f,", v);
    for (double v : {t_ctrl_start, t_ctrl_end, ack.t_sent, ack.t_plc_ack, ack.t_user3, ack.t_user4, ack.plc_raw_stamp})
      put(log, v);
    std::fprintf(log, "%d,%d,%.2f,%d,%s,%s\n", int(r.backup_active), int(r.intervened), z, zi, status, reason.c_str());
    std::fflush(log);

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

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--onnx") opt.onnx = true;
    else if (a == "--lab") opt.lab = true;
    else if (a == "--open-loop") opt.open_loop = true;
    else if (a == "--fast") opt.fast = true;
    else if (a == "--model-dir" && i + 1 < argc) opt.model_dir = argv[++i];
    else if (a == "--log" && i + 1 < argc) opt.log = argv[++i];
    else if (a == "--noise" && i + 1 < argc) opt.noise = std::atof(argv[++i]);
    else if (a == "--max-steps" && i + 1 < argc) opt.max_steps = std::atoi(argv[++i]);
    else if (a == "--sim-latency" && i + 1 < argc) opt.sim_latency = std::atof(argv[++i]);
    else if (a == "--sim-freeze-at" && i + 1 < argc) opt.sim_freeze_at = std::atoi(argv[++i]);
    else if (a == "--start" && i + 3 < argc) { for (int j = 0; j < 3; ++j) opt.start_hill[j] = std::atof(argv[++i]); }
    else {
      std::printf("usage: rail_main [--onnx] [--lab] [--open-loop] [--fast] [--max-steps N] [--model-dir DIR] "
                  "[--log FILE] [--noise M] [--start X Y Z] [--sim-latency S] [--sim-freeze-at K]\n");
      return a == "--help" ? 0 : 2;
    }
  }
#if defined(_WIN32) && defined(RVD_WINDOWS_TIMER)
  timeBeginPeriod(1);
#endif
  int rc = 2;
  try {
    rc = run(opt, RailConfig{});
  } catch (const std::exception& e) {
    std::fprintf(stderr, "fatal: %s\n", e.what());
  }
#if defined(_WIN32) && defined(RVD_WINDOWS_TIMER)
  timeEndPeriod(1);
#endif
  return rc;
}
