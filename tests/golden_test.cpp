// golden_test <repo_dir> [--onnx] [--trace RUN]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "pfcr/pfcr_controller.hpp"

using namespace pfcr;

namespace {

struct Csv {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  std::map<std::string, int> col;
  double num(size_t r, const std::string& c) const { return std::strtod(rows[r][col.at(c)].c_str(), nullptr); }
  const std::string& str(size_t r, const std::string& c) const { return rows[r][col.at(c)]; }
};

Csv read_csv(const std::string& path) {
  std::ifstream f(path);
  if (!f) { std::fprintf(stderr, "cannot open %s\n", path.c_str()); std::exit(2); }
  Csv c;
  std::string line;
  bool first = true;
  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) cells.push_back(cell);
    if (first) { c.header = cells; for (size_t i = 0; i < cells.size(); ++i) c.col[cells[i]] = int(i); first = false; }
    else c.rows.push_back(cells);
  }
  return c;
}

struct MaxDiff {
  double v = 0.0;
  void add(double a, double b) { v = std::fmax(v, std::fabs(a - b)); }
};

// replays reference actions in order
class ReplayPolicy final : public Policy {
 public:
  explicit ReplayPolicy(std::vector<Action> a) : a_(std::move(a)) {}
  Action act(const Obs&) override { return i_ < a_.size() ? a_[i_++] : Action{0, 0, 0}; }
  std::string name() const override { return "replay"; }
 private:
  std::vector<Action> a_;
  size_t i_ = 0;
};

std::unique_ptr<Policy> make_policy(const std::string& mdir, bool onnx) {
#ifdef PFCR_WITH_ONNXRUNTIME
  if (onnx) return make_onnx_policy(mdir + "/pfcr_policy.onnx");
#else
  if (onnx) { std::fprintf(stderr, "built without ONNX Runtime\n"); std::exit(2); }
#endif
  return make_builtin_policy(mdir + "/pfcr_policy_weights.bin");
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: golden_test <package_dir> [--onnx]\n"); return 2; }
  const std::string pkg = argv[1];
  bool onnx = false;
  int trace_run = -1;
  for (int i = 2; i < argc; ++i) {
    if (std::string(argv[i]) == "--onnx") onnx = true;
    if (std::string(argv[i]) == "--trace" && i + 1 < argc) trace_run = std::atoi(argv[++i]);
  }
  const std::string mdir = pkg + "/policies/" + params::kModelId;
  bool ok = true;

  {
    auto pol = make_policy(mdir, onnx);
    Csv c = read_csv(pkg + "/golden/policy_cases.csv");
    MaxDiff d;
    for (size_t r = 0; r < c.rows.size(); ++r) {
      Obs o;
      for (int i = 0; i < params::kObsDim; ++i) o[i] = static_cast<float>(c.num(r, "obs" + std::to_string(i)));
      const Action a = pol->act(o);
      for (int i = 0; i < 3; ++i) d.add(a[i], c.num(r, "a" + std::to_string(i)));
    }
    const bool pass = d.v <= 1e-5;
    ok &= pass;
    std::printf("[policy %s] %zu cases, max |action - reference| = %.3g  %s\n", pol->name().c_str(), c.rows.size(), d.v,
                pass ? "PASS" : "FAIL");
  }

  Csv meta = read_csv(pkg + "/golden/controller_runs_meta.csv");
  Csv g = read_csv(pkg + "/golden/controller_runs.csv");
  for (int pass_id = 0; pass_id < 2; ++pass_id) {
  const bool forced = pass_id == 0;
  std::printf(forced ? "-- (a) logic check, reference actions forced (tolerance 1e-9 N / m, float32 obs exact to 1e-6)\n"
                     : "-- (b) free-running closed loop with the %s policy (rounding drift is reported, not judged)\n",
              forced ? "" : (onnx ? "ONNX" : "built-in"));
  for (size_t mr = 0; mr < meta.rows.size(); ++mr) {
    const int run = int(meta.num(mr, "run"));
    ActuatorEmulation act;
    act.tau = meta.num(mr, "tau");
    act.scale = {meta.num(mr, "scale_x"), meta.num(mr, "scale_y"), meta.num(mr, "scale_z")};
    act.deadzone = meta.num(mr, "deadzone");
    std::vector<Action> ref_actions;
    for (size_t row = 0; row < g.rows.size(); ++row)
      if (int(g.num(row, "run")) == run && g.str(row, "status") == "RUN")
        ref_actions.push_back({float(g.num(row, "a0")), float(g.num(row, "a1")), float(g.num(row, "a2"))});
    Controller ctrl(forced ? std::unique_ptr<Policy>(new ReplayPolicy(ref_actions)) : make_policy(mdir, onnx), act);
    Vec6 s0{meta.num(mr, "x0"), meta.num(mr, "y0"), meta.num(mr, "z0"), meta.num(mr, "vx0"), meta.num(mr, "vy0"),
            meta.num(mr, "vz0")};
    ctrl.reset(s0);
    Vec3 pos{s0[0], s0[1], s0[2]};
    MaxDiff dobs, dact, dforce, dstate;
    int flag_mismatch = 0, steps = 0;
    StepResult r;
    std::string status_ref, reason_ref;
    int k_ref = -1;
    for (size_t row = 0; row < g.rows.size(); ++row) {
      if (int(g.num(row, "run")) != run) continue;
      r = ctrl.step(pos);
      ++steps;
      status_ref = g.str(row, "status");
      reason_ref = g.str(row, "reason");
      k_ref = int(g.num(row, "k"));
      for (int i = 0; i < 6; ++i) dstate.add(r.state[i], g.num(row, "s" + std::to_string(i)));
      if (status_ref != "RUN") break;
      for (int i = 0; i < params::kObsDim; ++i) dobs.add(r.obs[i], g.num(row, "obs" + std::to_string(i)));
      for (int i = 0; i < 3; ++i) {
        dact.add(r.action[i], g.num(row, "a" + std::to_string(i)));
        dforce.add(r.u_des[i], g.num(row, "ud" + std::to_string(i)));
        dforce.add(r.u_act[i], g.num(row, "ua" + std::to_string(i)));
        dforce.add(r.u_real[i], g.num(row, "ur" + std::to_string(i)));
      }
      for (int i = 0; i < 6; ++i) dstate.add(r.next_state[i], g.num(row, "n" + std::to_string(i)));
      flag_mismatch += int(r.backup_active) != int(g.num(row, "backup"));
      flag_mismatch += int(r.intervened) != int(g.num(row, "interv"));
      if (run == trace_run) {
        MaxDiff sa, sf, so;
        for (int i = 0; i < 3; ++i) {
          sa.add(r.action[i], g.num(row, "a" + std::to_string(i)));
          sf.add(r.u_act[i], g.num(row, "ua" + std::to_string(i)));
        }
        for (int i = 0; i < params::kObsDim; ++i) so.add(r.obs[i], g.num(row, "obs" + std::to_string(i)));
        std::printf("  k=%3d  |dobs| %.2e  |daction| %.2e  |du_act| %.2e  |dstate| %.2e  interv %d/%d  backup %d/%d\n",
                    r.k, so.v, sa.v, sf.v, dstate.v, int(r.intervened), int(g.num(row, "interv")),
                    int(r.backup_active), int(g.num(row, "backup")));
      }
      if (r.status != Status::Run) break;
      pos = {r.next_state[0], r.next_state[1], r.next_state[2]};
    }
    const bool same_end = status_ref == status_name(r.status) && reason_ref == r.reason && k_ref == r.k;
    const bool pass = same_end && flag_mismatch == 0 && dobs.v <= 1e-6 && dforce.v <= 1e-9 && dstate.v <= 1e-9;
    if (forced) ok &= pass;
    std::printf("[run %d %-26s] %4d steps  end %s/%s k=%d (ref %s/%s k=%d)  max|d| obs %.2g act %.2g force %.2g "
                "state %.2g  flag mismatches %d  %s\n",
                run, meta.str(mr, "name").c_str(), steps, status_name(r.status), r.reason.c_str(), r.k,
                status_ref.c_str(), reason_ref.c_str(), k_ref, dobs.v, dact.v, dforce.v, dstate.v, flag_mismatch,
                forced ? (pass ? "PASS" : "FAIL") : (same_end ? "same outcome" : "outcome differs (boundary drift)"));
  }
  }
  std::printf("GOLDEN TEST %s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
