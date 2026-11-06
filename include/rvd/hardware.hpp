#pragma once
#include "rvd/observations.hpp"
#include "rvd/rail_io.hpp"
namespace rvd {
// Contracts only: no SDK adapters, protocol implementations or hardware factory exist.
// All acquisitions must have bounded deadlines and timestamps mapped to the runtime clock.
class MotionCaptureInput {
 public:
  virtual ~MotionCaptureInput() = default;
  virtual bool read_pose(Pose& pose, double deadline_s) = 0;
};
class CameraInput {
 public:
  virtual ~CameraInput() = default;
  virtual bool read_frame(CameraFrame& frame, double deadline_s) = 0;
};
class LidarInput {
 public:
  virtual ~LidarInput() = default;
  virtual bool read_cloud(PointCloud& cloud, double deadline_s) = 0;
};
class TargetPlatform {
 public:
  virtual ~TargetPlatform() = default;
  virtual bool read_pose(Pose& pose, double deadline_s) = 0;
  virtual bool command_pose(const Pose& target, double deadline_s, CommandAck& ack) = 0;
  virtual void request_controlled_stop() noexcept = 0;
};
struct HardwareSafetyStatus {
  bool valid = false;  // unknown is unsafe
  bool emergency_stop = true;
  bool interlocks_ready = false;
  bool drives_ready = false;
  double acquired_s = kNoStamp;
};
class HardwareSafetyInput {
 public:
  virtual ~HardwareSafetyInput() = default;
  virtual HardwareSafetyStatus read_status(double deadline_s) = 0;
};
}  // namespace rvd
