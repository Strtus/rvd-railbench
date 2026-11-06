#pragma once
#include "rvd/observations.hpp"
namespace rvd {
struct ControllerInit {
  std::string artifact_directory;
  bool onnx = false;
};
enum class ControllerStatus { Uninitialized, Ready, Running, Completed, Fault };
struct ControllerOutput {
  ControllerStatus status = ControllerStatus::Uninitialized;
  std::optional<StateEstimate> reference;
  std::string reason;
};
struct ControllerDiagnostics {
  std::string type, version, backend;
  bool safety_intervened = false, backup_active = false;
  double minimum_margin = 0;
};
class IController {
 public:
  virtual ~IController() = default;
  virtual void initialize(const ControllerInit& settings) = 0;
  virtual void reset(const StateEstimate& initial) = 0;
  virtual ControllerOutput update(const Observation& observation) = 0;
  virtual ControllerStatus status() const = 0;
  virtual ControllerDiagnostics diagnostics() const = 0;
};
}  // namespace rvd
