#pragma once
#include <string>

namespace rvd {
// Offline model comparisons only; never sends commands to a rail or modifies a recording.
void write_preview(const std::string& experiment, const std::string& model, const std::string& output);
}  // namespace rvd
