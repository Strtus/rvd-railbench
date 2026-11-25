#include "rvd/external_peer.hpp"

#include <algorithm>
#include <stdexcept>
#include <string_view>

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
#include <fcntl.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#endif

namespace rvd {
namespace {
using Clock = std::chrono::steady_clock;
using Failure = ExternalPeer::Failure;
constexpr std::size_t kMaxLine = 512;

#ifdef _WIN32
using Native = SOCKET;
using SocketLength = int;
constexpr Native kInvalid = INVALID_SOCKET;

void close_native(Native socket) { closesocket(socket); }

bool nonblocking(Native socket) {
  u_long enabled = 1;
  return ioctlsocket(socket, FIONBIO, &enabled) == 0;
}

bool connect_pending() {
  const int error = WSAGetLastError();
  return error == WSAEWOULDBLOCK || error == WSAEINPROGRESS;
}

bool retry_io() {
  const int error = WSAGetLastError();
  return error == WSAEWOULDBLOCK || error == WSAEINTR;
}
#else
using Native = int;
using SocketLength = socklen_t;
constexpr Native kInvalid = -1;

void close_native(Native socket) { ::close(socket); }

bool nonblocking(Native socket) {
  const int flags = fcntl(socket, F_GETFL, 0);
  return flags >= 0 && fcntl(socket, F_SETFL, flags | O_NONBLOCK) == 0;
}

bool connect_pending() { return errno == EINPROGRESS || errno == EINTR; }
bool retry_io() { return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR; }
#endif

Failure wait_ready(Native socket, bool writing, Clock::time_point deadline) {
  for (;;) {
    const auto remaining = deadline - Clock::now();
    if (remaining <= Clock::duration::zero()) return Failure::Timeout;
#ifdef _WIN32
    const auto us = std::chrono::ceil<std::chrono::microseconds>(remaining).count();
    timeval timeout{static_cast<long>(us / 1000000), static_cast<long>(us % 1000000)};
    fd_set ready;
    fd_set errors;
    FD_ZERO(&ready);
    FD_ZERO(&errors);
    FD_SET(socket, &ready);
    FD_SET(socket, &errors);
    const int result = ::select(0, writing ? nullptr : &ready, writing ? &ready : nullptr, &errors, &timeout);
    if (result < 0 && WSAGetLastError() == WSAEINTR) continue;
#else
    const auto ms = std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
    pollfd descriptor{socket, static_cast<short>(writing ? POLLOUT : POLLIN), 0};
    const int result = ::poll(&descriptor, 1, static_cast<int>(ms));
    if (result < 0 && errno == EINTR) continue;
#endif
    if (result < 0) return Failure::Disconnected;
    return result == 0 || Clock::now() >= deadline ? Failure::Timeout : Failure::None;
  }
}

bool printable(std::string_view line) {
  return std::all_of(line.begin(), line.end(), [](unsigned char c) { return c >= 32 && c <= 126; });
}
}  // namespace

ExternalPeer::ExternalPeer(int port) {
  if (port <= 0 || port > 65535) throw std::invalid_argument("external port must be 1..65535");
#ifdef _WIN32
  WSADATA data{};
  if (WSAStartup(MAKEWORD(2, 2), &data) != 0) throw std::runtime_error("cannot start Winsock");
  winsock_ = true;
#endif
  const Native socket = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (socket == kInvalid) {
    disconnect();
    throw std::runtime_error("cannot create the external controller socket");
  }
  socket_ = static_cast<std::uintptr_t>(socket);
  connected_ = true;
  int enabled = 1;
  setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&enabled), sizeof enabled);
#ifdef SO_NOSIGPIPE
  if (setsockopt(socket, SOL_SOCKET, SO_NOSIGPIPE, &enabled, sizeof enabled) != 0) {
    disconnect();
    throw std::runtime_error("cannot disable SIGPIPE for the external controller socket");
  }
#endif
  if (!nonblocking(socket)) {
    disconnect();
    throw std::runtime_error("cannot make the external controller socket nonblocking");
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<std::uint16_t>(port));
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::connect(socket, reinterpret_cast<const sockaddr*>(&address), sizeof address) == 0) return;
  if (connect_pending() &&
      wait_ready(socket, true, Clock::now() + std::chrono::seconds(1)) == Failure::None) {
    int error = 0;
    SocketLength length = sizeof error;
    if (getsockopt(socket, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&error), &length) == 0 &&
        error == 0) {
      return;
    }
  }
  disconnect();
  throw std::runtime_error("cannot reach the external controller on loopback port " + std::to_string(port));
}

ExternalPeer::~ExternalPeer() { disconnect(); }

void ExternalPeer::disconnect() noexcept {
  if (connected_) close_native(static_cast<Native>(socket_));
  connected_ = false;
#ifdef _WIN32
  if (winsock_) WSACleanup();
#endif
  winsock_ = false;
}

Failure ExternalPeer::send_line(const std::string& line, Clock::time_point deadline) {
  if (!connected_) return Failure::Disconnected;
  if (line.empty() || line.size() > kMaxLine || !printable(line)) return Failure::Protocol;
  const Native socket = static_cast<Native>(socket_);
  const std::string wire = line + "\n";
  std::size_t sent = 0;
#ifdef MSG_NOSIGNAL
  constexpr int send_flags = MSG_NOSIGNAL;
#else
  constexpr int send_flags = 0;
#endif
  while (sent < wire.size()) {
    const auto wait = wait_ready(socket, true, deadline);
    if (wait != Failure::None) return wait;
    const auto count = ::send(socket, wire.data() + sent, static_cast<int>(wire.size() - sent), send_flags);
    if (count < 0 && retry_io()) continue;
    if (count <= 0) return Failure::Disconnected;
    sent += static_cast<std::size_t>(count);
  }
  return Failure::None;
}

bool ExternalPeer::exchange(const std::string& request, std::string& reply, Clock::time_point deadline) {
  reply.clear();
  failure_ = send_line(request, deadline);
  if (failure_ == Failure::None) {
    const Native socket = static_cast<Native>(socket_);
    while ((failure_ = wait_ready(socket, false, deadline)) == Failure::None) {
      char buffer[256];
      const auto count = ::recv(socket, buffer, static_cast<int>(sizeof buffer), 0);
      if (count < 0 && retry_io()) continue;
      if (count <= 0) {
        failure_ = Failure::Disconnected;
        break;
      }
      const std::string_view chunk(buffer, static_cast<std::size_t>(count));
      const auto end = chunk.find('\n');
      reply.append(chunk.substr(0, end));
      if (reply.size() > kMaxLine + 1) {
        failure_ = Failure::Protocol;
        break;
      }
      if (end == std::string_view::npos) continue;
      if (!reply.empty() && reply.back() == '\r') reply.pop_back();
      // A second, unsolicited response must not carry into another control tick.
      if (end + 1 != chunk.size() || reply.empty() || reply.size() > kMaxLine || !printable(reply))
        failure_ = Failure::Protocol;
      else if (Clock::now() >= deadline)
        failure_ = Failure::Timeout;
      else
        return true;
      break;
    }
  }
  disconnect();
  return false;
}

void ExternalPeer::notify(const std::string& line) noexcept {
  try {
    send_line(line, Clock::now() + std::chrono::milliseconds(50));
  } catch (...) {
  }
}
}  // namespace rvd
