#pragma once
#include "rvd/time.hpp"
#include "pfcr/pfcr_controller.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <variant>
#include <vector>
namespace rvd {
using pfcr::Vec3;
enum class Frame { Hill, Rail, MotionCapture, Camera, Lidar1, Lidar2, TargetBody };
struct StateEstimate {
  Frame frame = Frame::Hill;
  pfcr::Vec6 state{};  // metres, metres/second; Hill = radial, along-track, cross-track
  double acquired_s = kNoStamp;
};
struct Pose {
  Frame reference = Frame::MotionCapture;
  Vec3 position_m{};
  std::array<double,4> quaternion_wxyz{1,0,0,0};
  double acquired_s = kNoStamp;
};
struct CameraFrame {
  Frame frame = Frame::Camera;
  int width=0, height=0, stride_bytes=0;
  std::string encoding;
  double acquired_s=kNoStamp;
  std::shared_ptr<const std::vector<std::uint8_t>> pixels;
};
struct PointCloud {
  Frame frame = Frame::Lidar1;
  double acquired_s=kNoStamp;
  std::shared_ptr<const std::vector<Vec3>> points_m;
};
using SensorObservation = std::variant<Pose,CameraFrame,PointCloud>;
struct Observation {
  std::optional<StateEstimate> estimate;
  std::vector<SensorObservation> sensors;
};
}  // namespace rvd
