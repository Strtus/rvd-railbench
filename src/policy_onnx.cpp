// ONNX Runtime backend: input "obs" float32[1,36] (raw), output "action" float32[1,3].
#ifdef PFCR_WITH_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>

#include <stdexcept>
#include <string>

#include "pfcr/pfcr_controller.hpp"

namespace pfcr {
namespace {

#ifdef _WIN32
std::wstring to_ort_path(const std::string& s) { return std::wstring(s.begin(), s.end()); }
#else
std::string to_ort_path(const std::string& s) { return s; }
#endif

class OnnxPolicy final : public Policy {
 public:
  explicit OnnxPolicy(const std::string& path)
      : env_(ORT_LOGGING_LEVEL_WARNING, "pfcr"), session_(nullptr),
        mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
    Ort::SessionOptions opt;
    opt.SetIntraOpNumThreads(1);
    opt.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
    session_ = Ort::Session(env_, to_ort_path(path).c_str(), opt);
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1)
      throw std::runtime_error("unexpected ONNX signature");
  }
  std::string name() const override { return "onnxruntime"; }

  Action act(const Obs& obs) override {
    const int64_t in_shape[2] = {1, params::kObsDim};
    Ort::Value input = Ort::Value::CreateTensor<float>(mem_, const_cast<float*>(obs.data()), obs.size(), in_shape, 2);
    const char* in_names[] = {"obs"};
    const char* out_names[] = {"action"};
    auto out = session_.Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);
    const float* a = out.front().GetTensorData<float>();
    return {a[0], a[1], a[2]};
  }

 private:
  Ort::Env env_;
  Ort::Session session_;
  Ort::MemoryInfo mem_;
};

}  // namespace

std::unique_ptr<Policy> make_onnx_policy(const std::string& onnx_path) {
  return std::make_unique<OnnxPolicy>(onnx_path);
}

}  // namespace pfcr
#endif  // PFCR_WITH_ONNXRUNTIME
