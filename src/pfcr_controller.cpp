#include "pfcr/pfcr_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace pfcr {
namespace P = params;

const char* status_name(Status s) {
  switch (s) {
    case Status::Run: return "RUN";
    case Status::Docked: return "DOCKED";
    default: return "ABORT";
  }
}

namespace {

constexpr double kInvMass = 1.0 / P::kMass;

inline double norm3(double a, double b, double c) { return std::sqrt(a * a + b * b + c * c); }
inline double dot3(const Vec3& a, const Vec3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// continuous-time CWH drift A x
inline Vec6 cwh_Ax(const Vec6& x) {
  const double n = P::kN;
  return {x[3], x[4], x[5], 3.0 * (n * n) * x[0] + 2.0 * n * x[4], -2.0 * n * x[3], -(n * n) * x[2]};
}

struct Row { Vec3 g; double b; };   // g . u >= b

// ---------------------------------------------------------------- barrier rows
double speed_h(const Vec6& x) {
  return P::kNu0 + P::kNu1 * norm3(x[0], x[1], x[2]) - norm3(x[3], x[4], x[5]);
}
Row speed_row(const Vec6& x) {
  const double nu1 = P::kNu1, nu0 = P::kNu0, n = P::kN;
  const double d = 2.0 * (nu1 * nu1);
  double g[6] = {d * x[0], d * x[1], d * x[2], -2.0 * x[3], -2.0 * x[4], -2.0 * x[5]};
  const double r = norm3(x[0], x[1], x[2]) + 1e-12;
  for (int i = 0; i < 3; ++i) g[i] += 2.0 * nu1 * nu0 * x[i] / r;
  const double gA[6] = {g[3] * (3.0 * (n * n)), 0.0, g[5] * (-(n * n)), g[0] + g[4] * (-2.0 * n),
                        g[1] + g[3] * (2.0 * n), g[2]};
  double gAx = 0.0;
  for (int j = 0; j < 6; ++j) gAx += gA[j] * x[j];
  const double h = speed_h(x);
  const double alpha = 0.05 * h + 0.1 * std::pow(h, 3.0);
  return {{g[3] * kInvMass, g[4] * kInvMass, g[5] * kInvMass}, -gAx - alpha};
}
double axis_h(const Vec6& x, int i) { return P::kMaxVel * P::kMaxVel - x[3 + i] * x[3 + i]; }
Row axis_row(const Vec6& x, int i) {
  const double n = P::kN;
  const double gi = -2.0 * x[3 + i];
  double gAx;
  if (i == 0) gAx = (gi * (3.0 * (n * n))) * x[0] + (gi * (2.0 * n)) * x[4];
  else if (i == 1) gAx = (gi * (-2.0 * n)) * x[3];
  else gAx = (gi * (-(n * n))) * x[2];
  const double h = axis_h(x, i);
  const double alpha = 0.0005 * h + 0.001 * std::pow(h, 3.0);
  Vec3 g{0.0, 0.0, 0.0};
  g[i] = gi * kInvMass;
  return {g, -gAx - alpha};
}
std::array<double, 4> margins(const Vec6& x) { return {speed_h(x), axis_h(x, 0), axis_h(x, 1), axis_h(x, 2)}; }

// ---------------------------------------------------------------- QP: Euclidean projection, active-set enumeration
bool project(const Vec3& ud, const std::vector<Row>& rows, Vec3& out) {
  const double tol = 1e-10;
  auto feasible = [&](const Vec3& u) {
    for (const Row& r : rows) if (dot3(r.g, u) - r.b < -tol) return false;
    return true;
  };
  bool ud_ok = true;
  for (const Row& r : rows) if (dot3(r.g, ud) - r.b < 0.0) { ud_ok = false; break; }
  if (ud_ok) { out = ud; return true; }
  const int m = static_cast<int>(rows.size());
  double best = std::numeric_limits<double>::infinity();
  auto consider = [&](const int* idx, int s) {
    double G[3][3], rhs[3], lam[3] = {0, 0, 0};
    for (int a = 0; a < s; ++a) {
      for (int c = 0; c < 3; ++c) G[a][c] = rows[idx[a]].g[c];
      rhs[a] = rows[idx[a]].b - dot3(rows[idx[a]].g, ud);
    }
    double M[3][3];
    for (int a = 0; a < s; ++a)
      for (int b = 0; b < s; ++b) M[a][b] = G[a][0] * G[b][0] + G[a][1] * G[b][1] + G[a][2] * G[b][2];
    double scale = 1.0;
    for (int a = 0; a < s; ++a) scale *= M[a][a];
    if (s == 1) {
      if (M[0][0] <= 1e-24) return;
      lam[0] = rhs[0] / M[0][0];
    } else if (s == 2) {
      const double det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
      if (std::fabs(det) <= 1e-12 * scale || scale <= 1e-48) return;
      lam[0] = (rhs[0] * M[1][1] - M[0][1] * rhs[1]) / det;
      lam[1] = (M[0][0] * rhs[1] - M[1][0] * rhs[0]) / det;
    } else {
      const double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
                         M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
                         M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
      if (std::fabs(det) <= 1e-12 * scale || scale <= 1e-72) return;
      double inv[3][3];
      inv[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / det;
      inv[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / det;
      inv[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / det;
      inv[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) / det;
      inv[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / det;
      inv[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) / det;
      inv[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / det;
      inv[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) / det;
      inv[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / det;
      for (int a = 0; a < 3; ++a) lam[a] = inv[a][0] * rhs[0] + inv[a][1] * rhs[1] + inv[a][2] * rhs[2];
    }
    for (int a = 0; a < s; ++a) if (lam[a] < -1e-12) return;
    Vec3 u = ud;
    for (int a = 0; a < s; ++a)
      for (int c = 0; c < 3; ++c) u[c] += G[a][c] * lam[a];
    if (!feasible(u)) return;
    const double d = (u[0] - ud[0]) * (u[0] - ud[0]) + (u[1] - ud[1]) * (u[1] - ud[1]) + (u[2] - ud[2]) * (u[2] - ud[2]);
    if (d < best) { best = d; out = u; }
  };
  int idx[3];
  for (int i = 0; i < m; ++i) { idx[0] = i; consider(idx, 1); }
  for (int i = 0; i < m; ++i) for (int j = i + 1; j < m; ++j) { idx[0] = i; idx[1] = j; consider(idx, 2); }
  for (int i = 0; i < m; ++i) for (int j = i + 1; j < m; ++j) for (int l = j + 1; l < m; ++l) {
    idx[0] = i; idx[1] = j; idx[2] = l; consider(idx, 3);
  }
  return best < std::numeric_limits<double>::infinity();
}

// false when the rows are inconsistent
bool cbf_filter(const Vec6& x, const Vec3& ud, const Row& extra, Vec3& u_act) {
  std::vector<Row> rows = {
      {{1, 0, 0}, -P::kUMax}, {{-1, 0, 0}, -P::kUMax}, {{0, 1, 0}, -P::kUMax},
      {{0, -1, 0}, -P::kUMax}, {{0, 0, 1}, -P::kUMax}, {{0, 0, -1}, -P::kUMax},
      speed_row(x), axis_row(x, 0), axis_row(x, 1), axis_row(x, 2), extra};
  Vec3 u;
  if (!project(ud, rows, u)) return false;
  for (double& v : u) v = std::clamp(v, -P::kUMax, P::kUMax);
  for (const Row& r : rows) if (!std::isfinite(u[0] + u[1] + u[2]) || dot3(r.g, u) - r.b < -1e-8) return false;
  u_act = u;
  return true;
}

// ---------------------------------------------------------------- backup recovery guard
struct Geometry { Row command_row; Row physical_row; double distance, psi; bool in_scope, recoverable; };

class Guard {
 public:
  Guard() {
    const double taus[2] = {P::kTauMin, P::kTauMax}, scales[2] = {P::kScaleMin, P::kScaleMax};
    for (double tau : taus)
      for (double s : scales) {
        const double a = P::kDt / (P::kDt + tau);
        bool dup = false;
        for (auto& v : vertices_) dup |= (v[0] == a && v[1] == s);
        if (!dup) vertices_.push_back({a, s});
      }
    std::sort(vertices_.begin(), vertices_.end());
    dz_ = P::kDeadzoneMax;
    position_bound_ = P::kLeaveRadius + P::kDt * P::kMaxVel;
    velocity_bound_ = P::kMaxVel;
    drift_bound_ = 3.0 * (P::kN * P::kN) * position_bound_ + 2.0 * P::kN * velocity_bound_;
    a_min_ = vertices_.front()[0];
    for (auto& v : vertices_) a_min_ = std::min(a_min_, v[0]);
    rho_ = 1.0 - a_min_;
    double smin = vertices_.front()[1];
    for (auto& v : vertices_) smin = std::min(smin, v[1]);
    force_floor_ = smin * P::kUMax;
    acc_floor_ = force_floor_ / P::kMass - drift_bound_;
    if (acc_floor_ <= 0) throw std::runtime_error("outward backup cannot dominate the declared CWH drift");
    const int H = P::kGuardHorizon;
    k_.resize(H + 1); rk_.resize(H + 1); C_.resize(H + 1); B_.resize(H + 1);
    for (int i = 0; i <= H; ++i) {
      k_[i] = static_cast<double>(i);
      rk_[i] = std::pow(rho_, k_[i]);
      C_[i] = rho_ * (1.0 - rk_[i]) / (1.0 - rho_);
      B_[i] = rho_ / (1.0 - rho_) * (k_[i] - (1.0 - rk_[i]) / (1.0 - rho_));
    }
    reset();
  }
  void reset() { active_ = false; dwell_ = 0; hold_x_ = 3.0; ever_uncertified_ = false; }
  bool active() const { return active_; }

  bool in_scope(const Vec6& s, const Vec3& prev) const {
    const double vmax = std::max({std::fabs(s[3]), std::fabs(s[4]), std::fabs(s[5])});
    const double pmax = std::max({std::fabs(prev[0]), std::fabs(prev[1]), std::fabs(prev[2])});
    return std::fabs(s[0]) <= position_bound_ + 1e-8 && vmax <= velocity_bound_ + 1e-8 && pmax <= P::kUMax + 1e-8;
  }

  Geometry geometry(const Vec6& s, const Vec3& prev) const {
    for (double v : s) if (!std::isfinite(v)) throw std::runtime_error("non-finite backup input");
    for (double v : prev) if (!std::isfinite(v)) throw std::runtime_error("non-finite backup input");
    const double dt = P::kDt, m = P::kMass, ff = force_floor_;
    const double f0 = std::min(prev[0], ff);
    const int H = P::kGuardHorizon;
    double min_pos = std::numeric_limits<double>::infinity(), vel_last = 0, force_last = 0;
    for (int i = 0; i <= H; ++i) {
      const double force = ff + (f0 - ff) * rk_[i];
      const double velocity = s[3] + dt * (k_[i] * acc_floor_ + (f0 - ff) / m * C_[i]);
      const double position = s[0] + k_[i] * dt * s[3] +
                              (dt * dt) * (acc_floor_ * k_[i] * (k_[i] - 1.0) / 2.0 + (f0 - ff) / m * B_[i]);
      min_pos = std::min(min_pos, position);
      if (i == H) { vel_last = velocity; force_last = force; }
    }
    const double margin = min_pos - P::kFrontBuffer;
    const bool scope = in_scope(s, prev);
    bool recoverable = scope && margin >= -1e-8 && vel_last >= 0 && force_last / m >= drift_bound_;
    const double x1 = s[0] + dt * s[3];
    double required = -std::numeric_limits<double>::infinity();
    for (int i = 1; i <= H; ++i) {
      const double kk = k_[i], BB = B_[i];
      const double constant = x1 + kk * dt * s[3] +
                              (dt * dt) * (-kk * drift_bound_ + acc_floor_ * kk * (kk - 1.0) / 2.0 - ff / m * BB);
      required = std::max(required, (P::kFrontBuffer - constant) * m / ((dt * dt) * (kk + BB)));
    }
    const double terminal = s[3] - dt * drift_bound_ + dt * (H * acc_floor_ - ff / m * C_[H]);
    required = std::max(required, -terminal * m / (dt * (1.0 + C_[H])));
    recoverable = recoverable && required <= ff + 1e-8 && x1 >= P::kFrontBuffer - 1e-8;
    double output = -std::numeric_limits<double>::infinity();
    for (auto& v : vertices_) output = std::max(output, (required - (1.0 - v[0]) * prev[0]) / (v[0] * v[1]));
    const double lower = output > 0 ? std::max(output, dz_) : output;
    return {{{1, 0, 0}, lower + 1e-9}, {{1, 0, 0}, required}, s[0] - P::kFrontBuffer, margin, scope, recoverable};
  }

  bool command_certified(const Vec6& s, const Geometry& g, const Vec3& c) const {
    const double cmax = std::max({std::fabs(c[0]), std::fabs(c[1]), std::fabs(c[2])});
    return g.in_scope && g.recoverable && s[0] + P::kDt * s[3] >= P::kFrontBuffer - 1e-8 &&
           c[0] >= g.command_row.b - 1e-8 && cmax <= P::kUMax + 1e-8;
  }

  Vec3 select(const Vec6& s, const Vec3& desired, const Geometry& g) {
    Vec3 candidate{0, 0, 0};
    const bool solved = cbf_filter(s, desired, g.command_row, candidate);
    const bool admissible = solved && command_certified(s, g, candidate);
    const bool release_ok = admissible && g.psi >= P::kGuardReleaseMargin && dwell_ >= P::kGuardMinimumDwell &&
                            s[3] >= -0.02;
    const bool entered = !active_ && !admissible;
    const bool released = active_ && release_ok;
    if (entered) { active_ = true; dwell_ = 0; hold_x_ = std::max(s[0], 3.0); }
    else if (released) { active_ = false; dwell_ = 0; }
    Vec3 command;
    if (active_) {
      ++dwell_;
      const Vec6 Ax = cwh_Ax(s);
      const Vec3 ref{hold_x_, 0.0, 0.0};
      for (int i = 0; i < 3; ++i) command[i] = P::kMass * (0.01 * (ref[i] - s[i]) - 0.20 * s[3 + i] - Ax[3 + i]);
      if (s[3] < 0) command[0] = P::kUMax;
      for (double& v : command) v = std::clamp(v, -P::kUMax, P::kUMax);
      command[0] = std::min(P::kUMax, std::max(command[0], g.command_row.b));
      if (!g.recoverable) command[0] = P::kUMax;
    } else {
      command = candidate;
    }
    ever_uncertified_ |= !command_certified(s, g, command);
    return command;
  }

 private:
  std::vector<std::array<double, 2>> vertices_;
  double dz_, position_bound_, velocity_bound_, drift_bound_, a_min_, rho_, force_floor_, acc_floor_;
  std::vector<double> k_, rk_, C_, B_;
  bool active_ = false;
  int dwell_ = 0;
  double hold_x_ = 3.0;
  bool ever_uncertified_ = false;
};

// ---------------------------------------------------------------- thruster model
struct Thruster {
  ActuatorEmulation p;
  Vec3 u_prev{0, 0, 0};
  Vec3 apply(const Vec3& u_filt) {
    Vec3 u = u_filt;
    if (p.deadzone > 0.0) for (double& v : u) if (std::fabs(v) < p.deadzone) v = 0.0;
    for (int i = 0; i < 3; ++i) u[i] = p.scale[i] * u[i];
    if (p.tau > 1e-9) {
      const double a = P::kDt / (p.tau + P::kDt);
      for (int i = 0; i < 3; ++i) u[i] = u_prev[i] + a * (u[i] - u_prev[i]);
    }
    for (double& v : u) v = std::clamp(v, -P::kUMax, P::kUMax);
    u_prev = u;
    return u;
  }
};

}  // namespace

// ---------------------------------------------------------------- controller
struct Controller::Impl {
  std::unique_ptr<Policy> policy;
  VelocitySource vsrc;
  double tracking_tol;
  Guard guard;
  Thruster thruster;
  Vec6 x{};
  Vec3 prev_pos{};
  std::array<float, 3 * P::kHistoryLen> nom_hist{}, app_hist{};   // oldest first, / u_max
  float interv_flag = 0.0f, interv_mag = 0.0f;
  double time = 0.0;
  int k = 0;
  bool running = false;

  Obs build_obs() const {
    Obs o{};
    for (int i = 0; i < 3; ++i) o[i] = static_cast<float>(x[i] / P::kPosScale);
    for (int i = 0; i < 3; ++i) o[3 + i] = static_cast<float>(x[3 + i] / P::kVelScale);
    for (int i = 0; i < 12; ++i) o[6 + i] = nom_hist[i];
    for (int i = 0; i < 12; ++i) o[18 + i] = app_hist[i];
    const auto mg = margins(x);
    const double scale[4] = {1.0, P::kMaxVel * P::kMaxVel, P::kMaxVel * P::kMaxVel, P::kMaxVel * P::kMaxVel};
    for (int i = 0; i < 4; ++i) o[30 + i] = static_cast<float>(mg[i] / scale[i]);
    o[34] = interv_flag;
    o[35] = interv_mag;
    return o;
  }

  static void push(std::array<float, 3 * P::kHistoryLen>& h, const Vec3& v) {
    for (int i = 0; i < 3 * (P::kHistoryLen - 1); ++i) h[i] = h[i + 3];
    for (int i = 0; i < 3; ++i) h[3 * (P::kHistoryLen - 1) + i] = static_cast<float>(v[i] / P::kUMax);
  }

  StepResult abort(const char* reason, const Vec6* s) {
    running = false;
    StepResult r;
    r.status = Status::Abort;
    r.reason = reason;
    r.k = k;
    r.t = time;
    r.state = s ? *s : x;
    r.next_state = r.state;
    r.dist = norm3(r.state[0], r.state[1], r.state[2]);
    r.speed = norm3(r.state[3], r.state[4], r.state[5]);
    return r;
  }
};

Controller::Controller(std::unique_ptr<Policy> policy, ActuatorEmulation actuator, VelocitySource velocity_source,
                       double tracking_tol_m)
    : impl_(std::make_unique<Impl>()) {
  if (!policy) throw std::invalid_argument("policy is null");
  const auto& a = actuator;
  bool ok = a.tau >= P::kTauMin && a.tau <= P::kTauMax && a.deadzone >= P::kDeadzoneMin && a.deadzone <= P::kDeadzoneMax;
  for (double s : a.scale) ok = ok && s >= P::kScaleMin && s <= P::kScaleMax;
  if (!ok) throw std::invalid_argument("actuator emulation outside the box the safety guard is certified for");
  impl_->policy = std::move(policy);
  impl_->thruster.p = actuator;
  impl_->vsrc = velocity_source;
  impl_->tracking_tol = tracking_tol_m;
}
Controller::~Controller() = default;

bool Controller::running() const { return impl_->running; }
const Vec6& Controller::predicted_state() const { return impl_->x; }
const Policy& Controller::policy() const { return *impl_->policy; }

void Controller::reset(const Vec6& state0) {
  for (double v : state0) if (!std::isfinite(v)) throw std::invalid_argument("non-finite initial state");
  Impl& m = *impl_;
  m.guard.reset();
  m.thruster.u_prev = {0, 0, 0};
  m.nom_hist.fill(0.0f);
  m.app_hist.fill(0.0f);
  m.interv_flag = 0.0f;
  m.interv_mag = 0.0f;
  m.time = 0.0;
  m.x = state0;
  m.prev_pos = {state0[0], state0[1], state0[2]};
  m.k = 0;
  m.running = true;
}

StepResult Controller::step(const Vec3& pos, const Vec3* vel_meas) {
  Impl& m = *impl_;
  if (!m.running) throw std::logic_error("controller is not running; call reset() first");
  if (!std::isfinite(pos[0] + pos[1] + pos[2])) return m.abort("NONFINITE_MEASUREMENT", nullptr);
  const Vec6 predicted = m.x;
  Vec3 vel;
  if (m.k == 0) {
    vel = vel_meas ? *vel_meas : Vec3{predicted[3], predicted[4], predicted[5]};
  } else if (m.vsrc == VelocitySource::Measured) {
    if (!vel_meas) return m.abort("VELOCITY_REQUIRED", nullptr);
    vel = *vel_meas;
  } else if (m.vsrc == VelocitySource::FiniteDifference) {
    for (int i = 0; i < 3; ++i) vel[i] = (pos[i] - m.prev_pos[i]) / P::kDt;
  } else {
    vel = {predicted[3], predicted[4], predicted[5]};
  }
  Vec6 s{pos[0], pos[1], pos[2], vel[0], vel[1], vel[2]};
  if (m.k > 0 && norm3(pos[0] - predicted[0], pos[1] - predicted[1], pos[2] - predicted[2]) > m.tracking_tol)
    return m.abort("TRACKING_ERROR", &s);
  m.x = s;
  m.prev_pos = pos;
  const double dist = norm3(pos[0], pos[1], pos[2]), speed = norm3(vel[0], vel[1], vel[2]);
  if (dist <= P::kDockRadius && speed <= P::kDockSpeed && pos[0] >= P::kFrontBuffer - 1e-8) {
    m.running = false;
    StepResult r;
    r.status = Status::Docked;
    r.reason = "CAPTURED";
    r.k = m.k;
    r.t = m.time;
    r.state = s;
    r.next_state = s;
    r.dist = dist;
    r.speed = speed;
    return r;
  }
  if (pos[0] < P::kFrontBuffer - 1e-8) return m.abort("FRONT_PLANE_CROSSED", &s);
  if (m.time >= P::kMaxTime) return m.abort("TIMEOUT", &s);

  StepResult r;
  r.k = m.k;
  r.t = m.time;
  r.state = s;
  r.dist = dist;
  r.speed = speed;
  try {
    r.obs = m.build_obs();
    r.action = m.policy->act(r.obs);
    const Vec3 prev_real = m.thruster.u_prev;
    const Geometry geo = m.guard.geometry(s, prev_real);
    for (int i = 0; i < 3; ++i)
      r.u_des[i] = std::clamp(static_cast<double>(r.action[i]) * P::kUMax, -P::kUMax, P::kUMax);
    r.u_act = m.guard.select(s, r.u_des, geo);
    r.backup_active = m.guard.active();
    const double imag = norm3(r.u_act[0] - r.u_des[0], r.u_act[1] - r.u_des[1], r.u_act[2] - r.u_des[2]);
    r.intervened = imag > P::kInterveneTol;
    r.u_real = m.thruster.apply(r.u_act);
    const Vec6 Ax = cwh_Ax(s);
    for (int i = 0; i < 6; ++i) {
      const double bu = i < 3 ? 0.0 : kInvMass * r.u_real[i - 3];
      r.next_state[i] = s[i] + (Ax[i] + bu) * P::kDt;
    }
    for (double v : r.next_state) if (!std::isfinite(v)) throw std::runtime_error("non-finite next state");
    m.time += P::kDt;
    Impl::push(m.nom_hist, r.u_des);
    Impl::push(m.app_hist, r.u_real);
    m.interv_flag = r.intervened ? 1.0f : 0.0f;
    m.interv_mag = static_cast<float>(imag);
    m.x = r.next_state;
    const auto mg = margins(r.next_state);
    r.min_margin = *std::min_element(mg.begin(), mg.end());
  } catch (const std::exception&) {
    return m.abort("CONTROLLER_ERROR", &s);
  }
  ++m.k;
  return r;
}

}  // namespace pfcr
