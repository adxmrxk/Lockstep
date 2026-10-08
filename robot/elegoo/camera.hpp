// robot/elegoo/camera.hpp : pull single JPEG frames off the car's camera.
//
// The V4 kit's ESP32-WROVER runs a second, ordinary HTTP server alongside the
// brace-protocol socket on port 100:
//
//     http://192.168.4.1/capture       one JPEG
//     http://192.168.4.1:81/stream     MJPEG, multipart/x-mixed-replace
//
// This takes the first one, a frame at a time, and that is a deliberate choice
// rather than the easy one.
//
// WHY SINGLE SHOTS AND NOT THE STREAM
//
// The control loop runs at 4 Hz, because one command round trip to this car
// costs about 250 ms. A 10 fps stream would therefore deliver six frames a
// second that belong to no decision, and pairing a frame with the decision it
// belongs to would mean timestamp correlation -- reintroducing exactly the kind
// of timing dependency this project exists to make reproducible. Pulling one
// frame per cycle makes "frame N" and "decision N" the same event by
// construction, and nothing has to be correlated afterwards.
//
// It is also kinder to the ESP32, which is already the prime suspect for the
// link dying mid-drive, and it needs no multipart parser.
//
// If 4 fps ever proves too coarse, moving to the stream is a change confined to
// this header: everything downstream takes frames off a topic and does not care
// where they came from.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <string>

#include "lockstep/net/tcp.hpp"

namespace ls::camera {

struct options {
  std::string host = "192.168.4.1";
  std::uint16_t port = 80;
  std::string path = "/capture";
  int connect_timeout_ms = 1500;
  int read_timeout_ms = 2000;
  // The bus slot a frame travels in. A larger frame is refused here rather than
  // truncated, because half a JPEG is not a smaller picture, it is a corrupt
  // one that the video writer would then have to reject anyway.
  std::uint32_t max_bytes = 65520;
};

struct result {
  bool ok = false;
  std::string body;        // the JPEG, when ok
  std::string error;       // why not, when !ok
};

// One HTTP GET, one JPEG, connection closed.
//
// A fresh connection per frame rather than keep-alive: at 4 Hz the handshake is
// noise next to the 250 ms the car takes to answer anything, and the ESP32's
// HTTP server is happier closing sockets than holding them. It also means a
// frame that times out cannot leave a half-read body poisoning the next one,
// which is the bug that cost this project a day on the command socket.
inline result fetch_jpeg(const options& o) {
  result r;
  std::string err;
  ls::net::tcp_stream s =
      ls::net::tcp_stream::connect(o.host, o.port, o.connect_timeout_ms, &err);
  if (!s.valid()) {
    r.error = err.empty() ? "connect failed" : err;
    return r;
  }

  const std::string req = "GET " + o.path + " HTTP/1.1\r\nHost: " + o.host +
                          "\r\nConnection: close\r\nAccept: image/jpeg\r\n\r\n";
  if (!s.send_all(req)) {
    r.error = "send failed";
    return r;
  }

  // --- status line and headers ---
  //
  // Read line by line rather than slurping, because the body is binary and must
  // not be split on \n. recv_until leaves the \n consumed, so a bare \r is what
  // an empty line looks like here.
  std::string line;
  if (!s.recv_until('\n', line, o.read_timeout_ms)) {
    r.error = "no response";
    return r;
  }
  if (line.find(" 200") == std::string::npos) {
    // Trim the trailing \r so the message reads cleanly in a log.
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
      line.pop_back();
    r.error = line.empty() ? "bad status line" : line;
    return r;
  }

  long long declared = -1;
  for (;;) {
    if (!s.recv_until('\n', line, o.read_timeout_ms)) {
      r.error = "headers truncated";
      return r;
    }
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n'))
      line.pop_back();
    if (line.empty()) break;  // end of headers

    // Header names are case insensitive and the ESP32 sends "Content-Length".
    // Compare lowercased rather than trusting the capitalisation.
    std::string lower;
    lower.reserve(line.size());
    for (const char c : line)
      lower.push_back(static_cast<char>(c >= 'A' && c <= 'Z' ? c + 32 : c));
    if (lower.rfind("content-length:", 0) == 0) {
      declared = std::atoll(line.c_str() + 15);
    }
  }

  if (declared > static_cast<long long>(o.max_bytes)) {
    r.error = "frame is " + std::to_string(declared) + " bytes, over the " +
              std::to_string(o.max_bytes) + " byte slot";
    return r;
  }

  // --- body ---
  //
  // With a Content-Length, read exactly that much. Without one the server
  // signals the end by closing, so read until it does -- the ESP32 firmware has
  // been seen doing both depending on version.
  char buf[4096];
  if (declared >= 0) {
    r.body.reserve(static_cast<std::size_t>(declared));
    while (r.body.size() < static_cast<std::size_t>(declared)) {
      const std::size_t want =
          static_cast<std::size_t>(declared) - r.body.size();
      const int k = s.recv_some(buf, want < sizeof buf ? want : sizeof buf,
                                o.read_timeout_ms);
      if (k <= 0) {
        r.error = "body truncated at " + std::to_string(r.body.size()) + " of " +
                  std::to_string(declared) + " bytes";
        return r;
      }
      r.body.append(buf, static_cast<std::size_t>(k));
    }
  } else {
    for (;;) {
      const int k = s.recv_some(buf, sizeof buf, o.read_timeout_ms);
      if (k <= 0) break;  // clean close ends the body
      if (r.body.size() + static_cast<std::size_t>(k) > o.max_bytes) {
        r.error = "frame exceeds the " + std::to_string(o.max_bytes) +
                  " byte slot";
        return r;
      }
      r.body.append(buf, static_cast<std::size_t>(k));
    }
  }

  // A JPEG starts FFD8. Checking here means a captive portal's HTML error page
  // served with a 200 is caught at the source instead of becoming an unplayable
  // frame several layers downstream.
  if (r.body.size() < 4 || static_cast<std::uint8_t>(r.body[0]) != 0xFF ||
      static_cast<std::uint8_t>(r.body[1]) != 0xD8) {
    r.error = "response is not a JPEG (" + std::to_string(r.body.size()) +
              " bytes)";
    r.body.clear();
    return r;
  }

  r.ok = true;
  return r;
}

}  // namespace ls::camera
