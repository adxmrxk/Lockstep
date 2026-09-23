// net/tcp.hpp : a very small blocking TCP client, and a listener for tests.
//
// This is the one place in the project that talks to the outside world. It is
// deliberately tiny and deliberately blocking: the bridge node that uses it runs
// off the real-time path, and a robot's WiFi link is going to dominate any
// latency here by four orders of magnitude. Nothing clever is warranted.
//
// It exists at all because the bridge has to reach a physical robot, and the
// rest of the bus must stay dependency-free -- so the sockets live here, behind
// an interface small enough to read in one sitting, rather than pulling a
// networking library into a project whose whole premise is that it has none.
//
// Both backends are present because the bus itself now runs on both platforms
// and it would be strange for the robot bridge to be the one thing that does
// not.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace ls::net {

#if defined(_WIN32)
using socket_t = SOCKET;
inline constexpr socket_t invalid_socket = INVALID_SOCKET;
#else
using socket_t = int;
inline constexpr socket_t invalid_socket = -1;
#endif

// Winsock needs a process-wide startup call; POSIX needs nothing. A function
// static makes it happen exactly once, on first use, with no init order
// problem and nothing for the caller to remember.
inline void init_once() {
#if defined(_WIN32)
  struct starter {
    starter() {
      WSADATA d{};
      ::WSAStartup(MAKEWORD(2, 2), &d);
    }
    ~starter() { ::WSACleanup(); }
  };
  static starter s;
  (void)s;
#endif
}

inline void close_socket(socket_t s) noexcept {
  if (s == invalid_socket) return;
#if defined(_WIN32)
  ::closesocket(s);
#else
  ::close(s);
#endif
}

inline std::string last_error() {
#if defined(_WIN32)
  return "winsock error " + std::to_string(::WSAGetLastError());
#else
  return std::string(std::strerror(errno));
#endif
}

// A connected TCP stream. Move-only, closes on destruction.
class tcp_stream {
 public:
  tcp_stream() = default;
  explicit tcp_stream(socket_t s) noexcept : sock_(s) {}

  tcp_stream(const tcp_stream&) = delete;
  tcp_stream& operator=(const tcp_stream&) = delete;
  tcp_stream(tcp_stream&& o) noexcept : sock_(o.sock_) { o.sock_ = invalid_socket; }
  tcp_stream& operator=(tcp_stream&& o) noexcept {
    if (this != &o) {
      close_socket(sock_);
      sock_ = o.sock_;
      o.sock_ = invalid_socket;
    }
    return *this;
  }
  ~tcp_stream() { close_socket(sock_); }

  bool valid() const noexcept { return sock_ != invalid_socket; }
  socket_t handle() const noexcept { return sock_; }

  void close() noexcept {
    close_socket(sock_);
    sock_ = invalid_socket;
  }

  // Connect to host:port. `timeout_ms` bounds the connect only; a robot that is
  // off should fail fast rather than hanging the node for the OS default of
  // two minutes.
  static tcp_stream connect(const std::string& host, std::uint16_t port,
                            int timeout_ms, std::string* error = nullptr) {
    init_once();

    ::addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    ::addrinfo* res = nullptr;
    const std::string port_s = std::to_string(port);
    if (::getaddrinfo(host.c_str(), port_s.c_str(), &hints, &res) != 0 || res == nullptr) {
      if (error) *error = "cannot resolve " + host;
      return tcp_stream();
    }

    socket_t s = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s == invalid_socket) {
      ::freeaddrinfo(res);
      if (error) *error = "socket(): " + last_error();
      return tcp_stream();
    }

    set_blocking(s, false);
    int rc = ::connect(s, res->ai_addr, static_cast<int>(res->ai_addrlen));
    ::freeaddrinfo(res);

    if (rc != 0 && !would_block()) {
      close_socket(s);
      if (error) *error = "connect(): " + last_error();
      return tcp_stream();
    }

    if (rc != 0 && !wait_writable(s, timeout_ms)) {
      close_socket(s);
      if (error) *error = "connect timed out after " + std::to_string(timeout_ms) + " ms";
      return tcp_stream();
    }

    // A non-blocking connect that "completes" can still have failed; SO_ERROR
    // is the only way to tell.
    int so_err = 0;
    socklen_t len = sizeof(so_err);
    ::getsockopt(s, SOL_SOCKET, SO_ERROR, reinterpret_cast<char*>(&so_err), &len);
    if (so_err != 0) {
      close_socket(s);
      if (error) *error = "connect failed (SO_ERROR=" + std::to_string(so_err) + ")";
      return tcp_stream();
    }

    set_blocking(s, true);
    // The robot protocol is small request/response messages. Nagle would batch
    // them and add up to 40 ms of latency to a control loop for no benefit.
    int one = 1;
    ::setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<char*>(&one), sizeof(one));
    return tcp_stream(s);
  }

  // Send everything or fail. Partial writes are a real thing on a slow link and
  // silently sending half a command would be worse than not sending it.
  bool send_all(const void* data, std::size_t n) {
    const char* p = static_cast<const char*>(data);
    std::size_t sent = 0;
    while (sent < n) {
      const int k = ::send(sock_, p + sent, static_cast<int>(n - sent), 0);
      if (k <= 0) return false;
      sent += static_cast<std::size_t>(k);
    }
    return true;
  }

  bool send_all(const std::string& s) { return send_all(s.data(), s.size()); }

  // Read whatever is available, up to `n`. Returns bytes read, 0 on clean
  // close, -1 on error or timeout.
  int recv_some(void* buf, std::size_t n, int timeout_ms) {
    if (timeout_ms >= 0 && !wait_readable(sock_, timeout_ms)) return -1;
    return ::recv(sock_, static_cast<char*>(buf), static_cast<int>(n), 0);
  }

  // Read until `delim` is seen. Used for the robot's newline-framed replies.
  // Returns false on timeout, close or overflow.
  bool recv_until(char delim, std::string& out, int timeout_ms,
                  std::size_t limit = 4096) {
    out.clear();
    char c = 0;
    while (out.size() < limit) {
      const int k = recv_some(&c, 1, timeout_ms);
      if (k <= 0) return false;
      if (c == delim) return true;
      out.push_back(c);
    }
    return false;
  }

  static bool wait_readable(socket_t s, int timeout_ms) {
    fd_set r;
    FD_ZERO(&r);
    FD_SET(s, &r);
    ::timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    return ::select(static_cast<int>(s) + 1, &r, nullptr, nullptr, &tv) > 0;
  }

  static bool wait_writable(socket_t s, int timeout_ms) {
    fd_set w;
    FD_ZERO(&w);
    FD_SET(s, &w);
    ::timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    return ::select(static_cast<int>(s) + 1, nullptr, &w, nullptr, &tv) > 0;
  }

 private:
  static void set_blocking(socket_t s, bool blocking) {
#if defined(_WIN32)
    u_long mode = blocking ? 0u : 1u;
    ::ioctlsocket(s, FIONBIO, &mode);
#else
    int flags = ::fcntl(s, F_GETFL, 0);
    ::fcntl(s, F_SETFL, blocking ? (flags & ~O_NONBLOCK) : (flags | O_NONBLOCK));
#endif
  }

  static bool would_block() {
#if defined(_WIN32)
    const int e = ::WSAGetLastError();
    return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS;
#else
    return errno == EINPROGRESS || errno == EWOULDBLOCK;
#endif
  }

  socket_t sock_ = invalid_socket;
};

// A minimal listener. Exists so the bridge's protocol handling can be tested
// against a real socket with no robot present -- tests/test_elegoo_bridge.cpp
// stands up a fake robot on loopback and drives the bridge against it.
class tcp_listener {
 public:
  tcp_listener() = default;
  tcp_listener(const tcp_listener&) = delete;
  tcp_listener& operator=(const tcp_listener&) = delete;
  ~tcp_listener() { close_socket(sock_); }

  // Binds to an ephemeral port on loopback; port() reports which.
  bool listen_loopback() {
    init_once();
    sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ == invalid_socket) return false;

    int one = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char*>(&one),
                 sizeof(one));

    ::sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ::htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;  // let the OS choose
    if (::bind(sock_, reinterpret_cast<::sockaddr*>(&addr), sizeof(addr)) != 0) return false;
    if (::listen(sock_, 4) != 0) return false;

    ::sockaddr_in bound{};
    socklen_t len = sizeof(bound);
    if (::getsockname(sock_, reinterpret_cast<::sockaddr*>(&bound), &len) != 0) return false;
    port_ = ::ntohs(bound.sin_port);
    return true;
  }

  std::uint16_t port() const noexcept { return port_; }

  tcp_stream accept(int timeout_ms) {
    if (!tcp_stream::wait_readable(sock_, timeout_ms)) return tcp_stream();
    socket_t c = ::accept(sock_, nullptr, nullptr);
    if (c == invalid_socket) return tcp_stream();
    int one = 1;
    ::setsockopt(c, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<char*>(&one), sizeof(one));
    return tcp_stream(c);
  }

 private:
  socket_t sock_ = invalid_socket;
  std::uint16_t port_ = 0;
};

}  // namespace ls::net
