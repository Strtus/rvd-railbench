// Actor MLP evaluated from pfcr_policy_weights.bin (same computation as pfcr_policy.onnx).
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "pfcr/pfcr_controller.hpp"

namespace pfcr {
namespace {

struct Blob {
  uint32_t obs_dim = 0, act_dim = 0, n_hidden = 0;
  std::vector<uint32_t> sizes;
  float clip_obs = 10.0f;
  std::vector<double> mean, std;
  std::vector<std::vector<float>> W, b;
};

Blob read_blob(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot open policy weights: " + path);
  char magic[4];
  uint32_t version = 0;
  Blob B;
  f.read(magic, 4);
  f.read(reinterpret_cast<char*>(&version), 4);
  f.read(reinterpret_cast<char*>(&B.obs_dim), 4);
  f.read(reinterpret_cast<char*>(&B.act_dim), 4);
  f.read(reinterpret_cast<char*>(&B.n_hidden), 4);
  if (!f || std::string(magic, 4) != "PFCR" || version != 1) throw std::runtime_error("bad policy weights header");
  if (B.obs_dim != params::kObsDim || B.act_dim != 3) throw std::runtime_error("policy dimensions do not match");
  if (B.n_hidden == 0 || B.n_hidden > 16) throw std::runtime_error("invalid hidden layer count");
  B.sizes.resize(B.n_hidden);
  f.read(reinterpret_cast<char*>(B.sizes.data()), 4 * B.n_hidden);
  for (auto size : B.sizes) if (size == 0 || size > 4096) throw std::runtime_error("invalid hidden layer width");
  f.read(reinterpret_cast<char*>(&B.clip_obs), 4);
  B.mean.resize(B.obs_dim);
  B.std.resize(B.obs_dim);
  f.read(reinterpret_cast<char*>(B.mean.data()), 8 * B.obs_dim);
  f.read(reinterpret_cast<char*>(B.std.data()), 8 * B.obs_dim);
  if (!f || !std::isfinite(B.clip_obs) || B.clip_obs <= 0) throw std::runtime_error("invalid observation normalization");
  for (size_t i = 0; i < B.mean.size(); ++i)
    if (!std::isfinite(B.mean[i]) || !std::isfinite(B.std[i]) || B.std[i] <= 0)
      throw std::runtime_error("invalid normalization statistics");
  uint32_t in = B.obs_dim;
  for (uint32_t l = 0; l <= B.n_hidden; ++l) {
    const uint32_t out = l < B.n_hidden ? B.sizes[l] : B.act_dim;
    B.W.emplace_back(static_cast<size_t>(out) * in);
    B.b.emplace_back(out);
    f.read(reinterpret_cast<char*>(B.W.back().data()), 4 * B.W.back().size());
    f.read(reinterpret_cast<char*>(B.b.back().data()), 4 * B.b.back().size());
    in = out;
  }
  if (!f) throw std::runtime_error("truncated policy weights: " + path);
  for (const auto& layer : B.W) for (float v : layer)
    if (!std::isfinite(v)) throw std::runtime_error("nonfinite policy weight");
  for (const auto& layer : B.b) for (float v : layer)
    if (!std::isfinite(v)) throw std::runtime_error("nonfinite policy bias");
  if (f.peek() != std::char_traits<char>::eof()) throw std::runtime_error("trailing policy data");
  return B;
}

class BuiltinPolicy final : public Policy {
 public:
  explicit BuiltinPolicy(const std::string& path) : B_(read_blob(path)) {}
  std::string name() const override { return "builtin-mlp"; }

  Action act(const Obs& obs) override {
    std::vector<float> h(B_.obs_dim);
    for (uint32_t i = 0; i < B_.obs_dim; ++i) {
      double z = (static_cast<double>(obs[i]) - B_.mean[i]) / B_.std[i];
      z = std::fmin(std::fmax(z, -static_cast<double>(B_.clip_obs)), static_cast<double>(B_.clip_obs));
      h[i] = static_cast<float>(z);
    }
    for (uint32_t l = 0; l <= B_.n_hidden; ++l) {
      const size_t in = h.size(), out = B_.b[l].size();
      std::vector<float> y(out);
      for (size_t j = 0; j < out; ++j) {
        double acc = 0.0;
        const float* w = &B_.W[l][j * in];
        for (size_t i = 0; i < in; ++i) acc += static_cast<double>(w[i]) * static_cast<double>(h[i]);
        acc += static_cast<double>(B_.b[l][j]);
        y[j] = static_cast<float>(l < B_.n_hidden ? std::tanh(static_cast<double>(static_cast<float>(acc))) : acc);
      }
      h.swap(y);
    }
    Action a{};
    for (int i = 0; i < 3; ++i) a[i] = std::fmin(std::fmax(h[i], -1.0f), 1.0f);
    return a;
  }

 private:
  Blob B_;
};

}  // namespace

std::unique_ptr<Policy> make_builtin_policy(const std::string& weights_bin_path) {
  return std::make_unique<BuiltinPolicy>(weights_bin_path);
}

bool ObsStats::load(const std::string& weights_bin_path) {
  try {
    const Blob B = read_blob(weights_bin_path);
    for (int i = 0; i < params::kObsDim; ++i) { mean[i] = B.mean[i]; std[i] = B.std[i]; }
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

double ObsStats::max_abs_z(const Obs& obs, int* index) const {
  double worst = 0.0;
  int at = 0;
  for (int i = 0; i < params::kObsDim; ++i) {
    const double z = std::fabs((static_cast<double>(obs[i]) - mean[i]) / std[i]);
    if (z > worst) { worst = z; at = i; }
  }
  if (index) *index = at;
  return worst;
}

}  // namespace pfcr
