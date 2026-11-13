// ONNX Runtime backend: input "obs" float32[1,obs_dim] (raw), output "action" float32[1,3].
#ifdef PFCR_WITH_ONNXRUNTIME
#include <onnxruntime_cxx_api.h>

#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "pfcr/pfcr_controller.hpp"

namespace pfcr {
namespace {

#ifdef _WIN32
std::wstring to_ort_path(const std::string& s) { return std::filesystem::u8path(s).wstring(); }
#else
std::string to_ort_path(const std::string& s) { return s; }
#endif

class OnnxPolicy final : public Policy {
 public:
  explicit OnnxPolicy(const std::string& path)
      : env_(ORT_LOGGING_LEVEL_WARNING, "pfcr"),
        session_(nullptr),
        mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
    Ort::SessionOptions opt;
    opt.SetIntraOpNumThreads(1);
    opt.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
    session_ = Ort::Session(env_, to_ort_path(path).c_str(), opt);
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1)
      throw std::runtime_error("unexpected ONNX signature");
    const auto input_type = session_.GetInputTypeInfo(0);
    const auto output_type = session_.GetOutputTypeInfo(0);
    const auto input = input_type.GetTensorTypeAndShapeInfo();
    const auto output = output_type.GetTensorTypeAndShapeInfo();
    const auto ins = input.GetShape(), outs = output.GetShape();
    if (input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || ins.size() != 2 ||
        outs.size() != 2 || (ins[0] != 1 && ins[0] != -1) || (outs[0] != 1 && outs[0] != -1) || ins[1] <= 0 ||
        ins[1] > 4096 || outs[1] != 3)
      throw std::runtime_error("ONNX dimensions or types do not match PFCR");
    dim_ = ins[1];
    Ort::AllocatorWithDefaultOptions allocator;
    const auto in_name = session_.GetInputNameAllocated(0, allocator);
    const auto out_name = session_.GetOutputNameAllocated(0, allocator);
    if (std::string(in_name.get()) != "obs" || std::string(out_name.get()) != "action")
      throw std::runtime_error("unexpected ONNX input/output names");
  }
  std::string name() const override { return "onnxruntime"; }
  int input_dim() const override { return static_cast<int>(dim_); }

  Action act(const Obs& obs) override {
    if (static_cast<int64_t>(obs.size()) != dim_)
      throw std::invalid_argument("observation size does not match the policy");
    const int64_t in_shape[2] = {1, dim_};
    Ort::Value input =
        Ort::Value::CreateTensor<float>(mem_, const_cast<float*>(obs.data()), obs.size(), in_shape, 2);
    const char* in_names[] = {"obs"};
    const char* out_names[] = {"action"};
    auto out = session_.Run(Ort::RunOptions{nullptr}, in_names, &input, 1, out_names, 1);
    if (out.size() != 1 || !out.front().IsTensor() ||
        out.front().GetTensorTypeAndShapeInfo().GetShape() != std::vector<int64_t>{1, 3})
      throw std::runtime_error("invalid ONNX output shape");
    const float* a = out.front().GetTensorData<float>();
    return {a[0], a[1], a[2]};
  }

 private:
  Ort::Env env_;
  Ort::Session session_;
  Ort::MemoryInfo mem_;
  int64_t dim_ = 0;
};

}  // namespace

std::unique_ptr<Policy> make_onnx_policy(const std::string& onnx_path) {
  return std::make_unique<OnnxPolicy>(onnx_path);
}

}  // namespace pfcr
#endif  // PFCR_WITH_ONNXRUNTIME
