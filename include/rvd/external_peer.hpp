#pragma once
#include <chrono>
#include <cstdint>
#include <string>

namespace rvd {
// One request and one reply per tick, over a loopback TCP connection.
class ExternalPeer {
 public:
  enum class Failure { None, Disconnected, Timeout, Protocol };

  explicit ExternalPeer(int port);
  ~ExternalPeer();
  ExternalPeer(const ExternalPeer&) = delete;
  ExternalPeer& operator=(const ExternalPeer&) = delete;

  bool exchange(const std::string& request, std::string& reply,
                std::chrono::steady_clock::time_point deadline);
  Failure failure() const { return failure_; }
  void notify(const std::string& line) noexcept;

 private:
  Failure send_line(const std::string& line, std::chrono::steady_clock::time_point deadline);
  void disconnect() noexcept;

  std::uintptr_t socket_ = 0;
  bool connected_ = false;
  bool winsock_ = false;
  Failure failure_ = Failure::None;
};
}  // namespace rvd
