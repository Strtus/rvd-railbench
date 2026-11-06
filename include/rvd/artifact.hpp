#pragma once
#include <string>
namespace rvd {
std::string sha256_file(const std::string& path);
void verify_artifact(const std::string& directory, bool onnx);
inline constexpr const char* kWeightsSha256 =
    "599c666e99f8bcbd95dc234f023f1d919717fedd652659444976d4d2bfaefdee";
inline constexpr const char* kOnnxSha256 = "9b39f7c79521f58d9f724b82d692e0a4b910bd70891fcd1135c4f9415e23f2ee";
}  // namespace rvd
