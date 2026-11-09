#include "rvd/bridge.hpp"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace rvd {
namespace {
#ifdef _WIN32
using Native = SOCKET;
const Native kInvalid = INVALID_SOCKET;
void close_native(Native s) { closesocket(s); }
#else
using Native = int;
const Native kInvalid = -1;
void close_native(Native s) { ::close(s); }
#endif
constexpr std::size_t kMaxLine = 65536;

std::vector<std::string> split(const std::string& line) {
  std::vector<std::string> words;
  std::size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() && line[i] == ' ') ++i;
    std::size_t j = i;
    while (j < line.size() && line[j] != ' ') ++j;
    if (j > i) words.push_back(line.substr(i, j - i));
    i = j;
  }
  return words;
}
bool number(const std::string& word, double& value) {
  char* end = nullptr;
  value = std::strtod(word.c_str(), &end);
  return end != word.c_str() && *end == '\0';
}
void report(const std::string& reply) {
  const auto text = reply.rfind("ERR ", 0) == 0 ? reply.substr(4) : "unexpected reply: " + reply;
  std::fprintf(stderr, "rail node: %s\n", text.c_str());
}
}  // namespace

BridgeRailIO::BridgeRailIO(int port) {
  if (port <= 0 || port > 65535) throw std::invalid_argument("bridge port must be 1..65535");
#ifdef _WIN32
  WSADATA data{};
  if (WSAStartup(MAKEWORD(2, 2), &data) != 0) throw std::runtime_error("cannot start Winsock");
  winsock_ = true;
#endif
  const Native s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == kInvalid) {
    disconnect();
    throw std::runtime_error("cannot create the bridge socket");
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<std::uint16_t>(port));
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::connect(s, reinterpret_cast<const sockaddr*>(&address), sizeof address) != 0) {
    close_native(s);
    disconnect();
    throw std::runtime_error("cannot reach the RVDLAB bridge on port " + std::to_string(port));
  }
  // A reply slower than this ends the run; the peer answers within one control period or reports an error.
#ifdef _WIN32
  const DWORD timeout_ms = 10000;
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout_ms), sizeof timeout_ms);
#else
  timeval timeout{10, 0};
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
#endif
  int on = 1;
  setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&on), sizeof on);
  socket_ = static_cast<std::uintptr_t>(s);
  connected_ = true;
}

BridgeRailIO::~BridgeRailIO() { disconnect(); }

void BridgeRailIO::disconnect() noexcept {
  if (connected_) close_native(static_cast<Native>(socket_));
  connected_ = false;
#ifdef _WIN32
  if (winsock_) WSACleanup();
#endif
  winsock_ = false;
}

bool BridgeRailIO::exchange(const std::string& request, std::string& reply) {
  if (!connected_) return false;
  const Native s = static_cast<Native>(socket_);
  const std::string line = request + "\n";
  std::size_t sent = 0;
  while (sent < line.size()) {
    const auto n = ::send(s, line.data() + sent, static_cast<int>(line.size() - sent), 0);
    if (n <= 0) {
      close_native(s);
      connected_ = false;
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }
  for (;;) {
    const auto end = buffer_.find('\n');
    if (end != std::string::npos) {
      reply = buffer_.substr(0, end);
      buffer_.erase(0, end + 1);
      if (!reply.empty() && reply.back() == '\r') reply.pop_back();
      return true;
    }
    char chunk[512];
    const auto n = ::recv(s, chunk, static_cast<int>(sizeof chunk), 0);
    if (n <= 0 || buffer_.size() > kMaxLine) {
      close_native(s);
      connected_ = false;
      return false;
    }
    buffer_.append(chunk, static_cast<std::size_t>(n));
  }
}

bool BridgeRailIO::read_measurement(Measurement& m) {
  std::string reply;
  if (!exchange("READ", reply)) return false;
  const double received = now_s();
  const auto words = split(reply);
  double v[10] = {};
  bool ok = words.size() == 11 && words[0] == "POS";
  for (int i = 0; ok && i < 10; ++i) ok = number(words[static_cast<std::size_t>(i) + 1], v[i]);
  if (!ok) {
    report(reply);
    return false;
  }
  stop_ = stop_ || v[9] != 0;
  for (int i = 0; i < 3; ++i) {
    m.rail_pos[i] = v[i];
    m.encoder_pos[i] = v[3 + i];
  }
  m.t_received = received;
  m.t_acquired = received - v[6];
  m.t_encoder = m.t_acquired;
  m.timestamp_uncertainty_s = v[7];
  m.synchronization = Synchronization::Estimated;
  m.frame_id = static_cast<long long>(v[8]);
  return true;
}

bool BridgeRailIO::send_target(const Command& c, CommandAck& ack) {
  char line[256];
  std::snprintf(line, sizeof line, "SEND %d %.9g %.9g %.9g %.9g %.9g %.9g", c.k, c.rail_target[0],
                c.rail_target[1], c.rail_target[2], c.rail_velocity[0], c.rail_velocity[1],
                c.rail_velocity[2]);
  ack.t_sent = now_s();
  std::string reply;
  if (!exchange(line, reply)) return false;
  const auto words = split(reply);
  double flag = 0;
  if (words.size() != 2 || words[0] != "ACK" || !number(words[1], flag)) {
    report(reply);
    return false;
  }
  ack.t_plc_ack = now_s();
  stop_ = stop_ || flag != 0;
  return true;
}

void BridgeRailIO::hold() noexcept {
  try {
    std::string reply;
    exchange("HOLD", reply);
  } catch (...) {
  }
}
}  // namespace rvd
