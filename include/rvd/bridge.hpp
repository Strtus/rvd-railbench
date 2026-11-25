#pragma once
#include <cstdint>
#include <memory>
#include <string>

#include "rvd/rail_io.hpp"
namespace rvd {
class ExternalPeer;
// Rail I/O relayed by a peer on a loopback TCP port (RVDLAB, which forwards to a rail node). One request
// line, one reply line:
//   READ                      -> POS rx ry rz ex ey ez age_s uncertainty_s frame stop | ERR text
//   SEND k tx ty tz vx vy vz  -> ACK stop | ERR text
//   HOLD                      -> OK
// External reference runs use SEND_UNTIL with an additional observation-to-deadline budget in seconds.
// rail = measured rail position, e = axis encoders, age_s = sample age when the reply is sent, stop = 0 or 1.
class BridgeRailIO final : public RailIO {
 public:
  explicit BridgeRailIO(int port, double external_timeout_s = 0);
  ~BridgeRailIO() override;
  BridgeRailIO(const BridgeRailIO&) = delete;
  BridgeRailIO& operator=(const BridgeRailIO&) = delete;
  bool read_measurement(Measurement& measurement) override;
  bool send_target(const Command& command, CommandAck& ack) override;
  void hold() noexcept override;
  bool stop_requested() override { return stop_; }

 private:
  bool exchange(const std::string& request, std::string& reply, double due = kNoStamp);
  void disconnect() noexcept;
  std::uintptr_t socket_ = 0;
  bool connected_ = false;
  bool winsock_ = false;
  bool stop_ = false;
  std::string buffer_;
  double external_timeout_s_ = 0, external_sample_upper_s_ = kNoStamp;
  std::unique_ptr<ExternalPeer> external_peer_;
};
}  // namespace rvd
