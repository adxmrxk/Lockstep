// robot/elegoo/protocol.hpp : the Elegoo Smart Robot Car V4 wire format.
//
// ===========================================================================
// READ THIS FIRST. This file contains the only claims in the project that were
// not verified against the thing they describe.
//
// I do not have the robot. The command numbers and field names below come from
// Elegoo's published firmware and app protocol for the V4 kit, not from a
// packet capture. Everything AROUND this file is tested -- the socket layer
// against a loopback server, the encoders against expected strings, the bridge
// against a fake robot -- but whether command 3 really means "move" on YOUR
// firmware is something only your robot can confirm.
//
// So the uncertainty is deliberately concentrated here, and made cheap to fix:
//
//   1. every tunable lives in `profile` below, as plain data;
//   2. `elegoo_bridge --probe` sends each command and prints the raw reply, so
//      you can see what your firmware actually does in about a minute;
//   3. if something is wrong, you edit one struct, not the bridge.
//
// If your kit shipped with different firmware, correct `profile` and everything
// above keeps working unchanged.
// ===========================================================================
//
// The transport: the ESP32 module on the V4 exposes a TCP socket (port 100 by
// default) and forwards whatever it receives to the UNO over serial. Commands
// are small JSON objects. Replies are short ASCII, usually brace-wrapped.
#pragma once

#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>

namespace ls::elegoo {

// ---------------------------------------------------------------------------
// The tunables. Edit these if --probe disagrees with them.
// ---------------------------------------------------------------------------
struct profile {
  // Command selectors, the "N" field.
  int cmd_motor = 1;        // per-motor drive
  int cmd_car_move = 3;     // whole-car direction + speed
  int cmd_servo = 5;        // camera/ultrasonic servo
  int cmd_ultrasonic = 21;  // range query
  int cmd_line_track = 22;  // IR line sensors
  int cmd_stop = 100;       // all stop
  int cmd_heartbeat = 110;  // keeps the firmware's failsafe from cutting out

  // For cmd_motor: which D1 selects which side.
  int motor_left = 1;
  int motor_right = 2;

  // Direction codes for cmd_car_move's D1.
  int dir_left = 1;
  int dir_right = 2;
  int dir_forward = 3;
  int dir_back = 4;

  // Speed is 0..255 in the stock firmware. Below ~80 the motors typically stall
  // rather than turning slowly, so the bridge clamps into a usable band instead
  // of sending values that do nothing.
  int speed_min = 0;
  int speed_max = 255;
  int speed_deadband = 80;

  // The firmware stops the car if it hears nothing for roughly 500 ms. The
  // bridge sends a heartbeat well inside that.
  int heartbeat_interval_ms = 200;
};

inline constexpr profile default_profile{};

// ---------------------------------------------------------------------------
// Encoding. Pure functions of their inputs, so they are unit-testable with no
// robot and no socket -- tests/test_elegoo_protocol.cpp pins every one.
// ---------------------------------------------------------------------------

inline int clamp_speed(int v, const profile& p) {
  if (v > p.speed_max) return p.speed_max;
  if (v < -p.speed_max) return -p.speed_max;
  return v;
}

// Map a requested speed onto what the motors can actually do. Anything inside
// the deadband becomes a real stop rather than a command the hardware silently
// ignores, which would otherwise look like a software bug.
inline int apply_deadband(int v, const profile& p) {
  if (v == 0) return 0;
  const int mag = v < 0 ? -v : v;
  if (mag < p.speed_deadband) return 0;
  return v;
}

inline std::string json_begin(int n) { return "{\"N\":" + std::to_string(n); }

inline std::string field(const char* key, int value) {
  return ",\"" + std::string(key) + "\":" + std::to_string(value);
}

// One side of the drivetrain. `speed` is signed: negative is reverse.
inline std::string encode_motor(int side_selector, int speed, const profile& p) {
  const int s = apply_deadband(clamp_speed(speed, p), p);
  const int mag = s < 0 ? -s : s;
  const int dir = s < 0 ? 0 : 1;
  return json_begin(p.cmd_motor) + field("D1", side_selector) + field("D2", mag) +
         field("D3", dir) + "}";
}

inline std::string encode_left(int speed, const profile& p = default_profile) {
  return encode_motor(p.motor_left, speed, p);
}
inline std::string encode_right(int speed, const profile& p = default_profile) {
  return encode_motor(p.motor_right, speed, p);
}

// Whole-car movement, which is what the stock app uses. Simpler and more likely
// to be correct on unmodified firmware than per-motor control.
enum class direction : int { forward, back, left, right };

inline std::string encode_move(direction d, int speed,
                               const profile& p = default_profile) {
  int code = p.dir_forward;
  switch (d) {
    case direction::forward: code = p.dir_forward; break;
    case direction::back: code = p.dir_back; break;
    case direction::left: code = p.dir_left; break;
    case direction::right: code = p.dir_right; break;
  }
  const int s = apply_deadband(clamp_speed(speed, p), p);
  const int mag = s < 0 ? -s : s;
  return json_begin(p.cmd_car_move) + field("D1", code) + field("D2", mag) + "}";
}

inline std::string encode_stop(const profile& p = default_profile) {
  return json_begin(p.cmd_stop) + "}";
}

inline std::string encode_heartbeat(const profile& p = default_profile) {
  return json_begin(p.cmd_heartbeat) + "}";
}

inline std::string encode_ultrasonic(const profile& p = default_profile) {
  return json_begin(p.cmd_ultrasonic) + field("D1", 2) + "}";
}

inline std::string encode_line_track(const profile& p = default_profile) {
  return json_begin(p.cmd_line_track) + field("D1", 0) + "}";
}

inline std::string encode_servo(int angle, const profile& p = default_profile) {
  int a = angle;
  if (a < 0) a = 0;
  if (a > 180) a = 180;
  return json_begin(p.cmd_servo) + field("D1", 1) + field("D2", a) + "}";
}

// ---------------------------------------------------------------------------
// Decoding.
//
// The firmware is loose about reply format -- observed shapes include {ok},
// {true}, {false}, and bare or underscore-wrapped numbers for sensor reads. So
// the parser is permissive on purpose: it pulls out the first integer it finds
// and reports whether the reply looked affirmative, rather than insisting on a
// grammar the firmware does not actually promise.
// ---------------------------------------------------------------------------
struct reply {
  bool affirmative = false;  // the reply looked like success
  bool has_value = false;    // an integer was present
  long value = 0;            // that integer (a distance in cm, typically)
  std::string raw;           // the untouched text, for --probe and for logs
};

inline reply parse_reply(std::string_view line) {
  reply r;
  r.raw.assign(line.begin(), line.end());

  // Strip the braces and whitespace the firmware sprinkles around replies.
  std::string t;
  t.reserve(line.size());
  for (const char c : line) {
    if (c == '{' || c == '}' || c == '_' || c == '\r' || c == '\n') continue;
    t.push_back(c);
  }

  // First integer, if any. A leading '-' counts.
  for (std::size_t i = 0; i < t.size(); ++i) {
    if ((t[i] >= '0' && t[i] <= '9') ||
        (t[i] == '-' && i + 1 < t.size() && t[i + 1] >= '0' && t[i + 1] <= '9')) {
      r.value = std::strtol(t.c_str() + i, nullptr, 10);
      r.has_value = true;
      break;
    }
  }

  // Affirmative if it says so, or if it is a bare number (a sensor answering).
  auto contains = [&t](const char* needle) {
    return t.find(needle) != std::string::npos;
  };
  if (contains("ok") || contains("OK") || contains("true") || contains("TRUE")) {
    r.affirmative = true;
  } else if (r.has_value && !contains("false") && !contains("FALSE")) {
    r.affirmative = true;
  }
  return r;
}

// An ultrasonic reading that is out of the sensor's believable range is a
// misread, not an obstacle at 3 metres. The HC-SR04 on this kit is specified to
// roughly 2..400 cm; anything outside that is discarded rather than fed to a
// controller that would steer on it.
inline constexpr long range_min_cm = 2;
inline constexpr long range_max_cm = 400;

inline bool range_is_plausible(long cm) {
  return cm >= range_min_cm && cm <= range_max_cm;
}

}  // namespace ls::elegoo
