#pragma once
#include <string>

#include "pfcr/pfcr_models.hpp"
namespace rvd {
std::string sha256_file(const std::string& path);
// The artifact must be the exact checkpoint the model's controller constants were ported for.
void verify_artifact(const std::string& directory, const pfcr::ModelSpec& model, bool onnx);
}  // namespace rvd
