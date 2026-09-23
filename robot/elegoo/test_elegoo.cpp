// Tests for the robot bridge, with no robot.
//
// Everything here runs on a desk with the car still in its box. Two halves:
//
//   1. the protocol encoders and the reply parser are pure functions, so they
//      are pinned exactly -- if an edit changes a command string, this notices;
//   2. the socket layer is driven against a FAKE ROBOT on loopback, which
//      speaks the replies the real firmware is documented to speak. That covers
//      connect, framing, timeout and disconnection.
//
// What this cannot cover, and does not pretend to: whether the real firmware
// agrees with robot/elegoo/protocol.hpp. Only the robot can answer that, which
// is what `elegoo_bridge --probe` is for.
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "lockstep/core/clock.hpp"
#include "lockstep/net/tcp.hpp"
#include "protocol.hpp"
#include "robot_msgs.hpp"
#include "support/check.hpp"

namespace {

using namespace ls::elegoo;

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

void commands_encode_exactly() {
  LS_CHECK(encode_stop() == std::string("{\"N\":100}"));
  LS_CHECK(encode_heartbeat() == std::string("{\"N\":110}"));
  LS_CHECK(encode_ultrasonic() == std::string("{\"N\":21,\"D1\":2}"));
  LS_CHECK(encode_move(direction::forward, 150) ==
           std::string("{\"N\":3,\"D1\":3,\"D2\":150}"));
  LS_CHECK(encode_move(direction::back, 150) ==
           std::string("{\"N\":3,\"D1\":4,\"D2\":150}"));
  LS_CHECK(encode_servo(90) == std::string("{\"N\":5,\"D1\":1,\"D2\":90}"));
}

// Reverse is a sign on the speed, encoded as a direction field plus magnitude.
// Getting this backwards would drive the car the wrong way, which is exactly
// the class of bug worth pinning.
void reverse_is_encoded_as_direction_zero() {
  const std::string fwd = encode_left(150);
  const std::string rev = encode_left(-150);
  LS_CHECK(fwd == std::string("{\"N\":1,\"D1\":1,\"D2\":150,\"D3\":1}"));
  LS_CHECK(rev == std::string("{\"N\":1,\"D1\":1,\"D2\":150,\"D3\":0}"));
  LS_CHECK(fwd != rev);
}

void left_and_right_select_different_motors() {
  LS_CHECK(encode_left(120) != encode_right(120));
  LS_CHECK(encode_left(120).find("\"D1\":1") != std::string::npos);
  LS_CHECK(encode_right(120).find("\"D1\":2") != std::string::npos);
}

// Below the deadband the motors stall rather than turning slowly, so a small
// request must become a real zero. Otherwise a controller asking for a gentle
// crawl gets silence and looks broken.
void speeds_below_the_deadband_become_zero() {
  const profile p;
  LS_CHECK_EQ(apply_deadband(0, p), 0);
  LS_CHECK_EQ(apply_deadband(p.speed_deadband - 1, p), 0);
  LS_CHECK_EQ(apply_deadband(-(p.speed_deadband - 1), p), 0);
  LS_CHECK_EQ(apply_deadband(p.speed_deadband, p), p.speed_deadband);
  LS_CHECK_EQ(apply_deadband(200, p), 200);
  LS_CHECK_EQ(apply_deadband(-200, p), -200);

  LS_CHECK(encode_left(10).find("\"D2\":0") != std::string::npos);
}

void speeds_are_clamped_to_the_hardware_range() {
  const profile p;
  LS_CHECK_EQ(clamp_speed(99999, p), p.speed_max);
  LS_CHECK_EQ(clamp_speed(-99999, p), -p.speed_max);
  LS_CHECK(encode_left(99999).find("\"D2\":255") != std::string::npos);
}

// ---------------------------------------------------------------------------
// Reply parsing. The firmware is loose about format, so the parser has to be
// permissive -- these are the shapes it is documented to produce.
// ---------------------------------------------------------------------------
void replies_parse() {
  const reply ok = parse_reply("{ok}");
  LS_CHECK(ok.affirmative);
  LS_CHECK(!ok.has_value);

  const reply t = parse_reply("{true}");
  LS_CHECK(t.affirmative);

  const reply f = parse_reply("{false}");
  LS_CHECK(!f.affirmative);

  const reply dist = parse_reply("{42}");
  LS_CHECK(dist.has_value);
  LS_CHECK_EQ(dist.value, 42L);
  LS_CHECK(dist.affirmative);

  const reply underscored = parse_reply("_137_");
  LS_CHECK(underscored.has_value);
  LS_CHECK_EQ(underscored.value, 137L);

  const reply crlf = parse_reply("{99}\r\n");
  LS_CHECK_EQ(crlf.value, 99L);

  const reply empty = parse_reply("");
  LS_CHECK(!empty.affirmative);
  LS_CHECK(!empty.has_value);

  // The raw text survives untouched, because --probe shows it to a human who is
  // trying to work out what their firmware does.
  LS_CHECK(parse_reply("{weird}").raw == std::string("{weird}"));
}

// A lost echo reads as 0 or as a number far past the sensor's range. Feeding
// either to a controller makes the car swerve at nothing.
void implausible_ranges_are_rejected() {
  LS_CHECK(!range_is_plausible(0));
  LS_CHECK(!range_is_plausible(1));
  LS_CHECK(!range_is_plausible(5000));
  LS_CHECK(!range_is_plausible(-3));
  LS_CHECK(range_is_plausible(2));
  LS_CHECK(range_is_plausible(50));
  LS_CHECK(range_is_plausible(400));
}

// ---------------------------------------------------------------------------
// The socket layer, against a fake robot.
// ---------------------------------------------------------------------------

// Speaks just enough Elegoo to exercise the bridge's link handling: newline
// framed, answers range queries with a number, everything else with {ok}.
struct fake_robot {
  ls::net::tcp_listener listener;
  std::thread thread;
  std::atomic<bool> running{false};
  std::atomic<int> commands_seen{0};
  long range_to_report = 57;

  bool start() {
    if (!listener.listen_loopback()) return false;
    running.store(true);
    thread = std::thread([this] {
      ls::net::tcp_stream c = listener.accept(3000);
      if (!c.valid()) return;
      std::string line;
      while (running.load() && c.recv_until('\n', line, 1000)) {
        commands_seen.fetch_add(1);
        const std::string resp =
            line.find("\"N\":21") != std::string::npos
                ? "{" + std::to_string(range_to_report) + "}\n"
                : std::string("{ok}\n");
        if (!c.send_all(resp)) break;
      }
    });
    return true;
  }

  void stop() {
    running.store(false);
    if (thread.joinable()) thread.join();
  }

  std::uint16_t port() const { return listener.port(); }
};

void the_link_talks_to_a_fake_robot() {
  fake_robot robot;
  LS_CHECK(robot.start());

  std::string err;
  ls::net::tcp_stream c =
      ls::net::tcp_stream::connect("127.0.0.1", robot.port(), 2000, &err);
  LS_CHECK(c.valid());
  if (!c.valid()) {
    std::fprintf(stderr, "  connect failed: %s\n", err.c_str());
    robot.stop();
    return;
  }

  // A range query round-trips and parses to the value the robot reported.
  LS_CHECK(c.send_all(encode_ultrasonic() + "\n"));
  std::string line;
  LS_CHECK(c.recv_until('\n', line, 2000));
  const reply r = parse_reply(line);
  LS_CHECK(r.has_value);
  LS_CHECK_EQ(r.value, 57L);
  LS_CHECK(range_is_plausible(r.value));

  // A drive command is acknowledged.
  LS_CHECK(c.send_all(encode_left(150) + "\n"));
  LS_CHECK(c.recv_until('\n', line, 2000));
  LS_CHECK(parse_reply(line).affirmative);

  LS_CHECK(robot.commands_seen.load() >= 2);
  c.close();
  robot.stop();
}

// Connecting to a port nobody is listening on must fail promptly rather than
// hanging the node for the OS default, which is around two minutes.
void connecting_to_nothing_fails_fast() {
  const std::uint64_t t0 = ls::now_ns();
  std::string err;
  // Port 1 on loopback: reserved, and nothing will be bound there.
  ls::net::tcp_stream c = ls::net::tcp_stream::connect("127.0.0.1", 1, 500, &err);
  const std::uint64_t took_ms = (ls::now_ns() - t0) / 1000000u;

  LS_CHECK(!c.valid());
  LS_CHECK(!err.empty());
  LS_CHECK(took_ms < 3000);  // generous, but nowhere near the OS default
}

// If the robot goes away mid-session the bridge has to notice, not block.
void a_dead_robot_is_detected() {
  fake_robot robot;
  LS_CHECK(robot.start());

  std::string err;
  ls::net::tcp_stream c =
      ls::net::tcp_stream::connect("127.0.0.1", robot.port(), 2000, &err);
  LS_CHECK(c.valid());

  robot.stop();  // the car is switched off

  // Reads must stop returning data rather than hanging forever.
  std::string line;
  bool got = c.recv_until('\n', line, 300);
  LS_CHECK(!got);
}

// ---------------------------------------------------------------------------
// The message types
// ---------------------------------------------------------------------------
void robot_messages_are_replay_safe() {
  // Padding is indeterminate and a replay hash covers the whole slot, so these
  // being padding-free is a correctness property, not a cosmetic one.
  LS_CHECK(!ls::message_traits<RobotRange>::has_padding);
  LS_CHECK(!ls::message_traits<RobotDrive>::has_padding);
  LS_CHECK(!ls::message_traits<RobotStatus>::has_padding);

  LS_CHECK(ls::message_traits<RobotRange>::declared);
  LS_CHECK(ls::message_traits<RobotDrive>::declared);
  LS_CHECK(ls::message_traits<RobotStatus>::declared);

  // Distinct types must not be confusable at connect time.
  LS_CHECK(ls::message_traits<RobotRange>::layout_hash !=
           ls::message_traits<RobotDrive>::layout_hash);
}

}  // namespace

int main() {
  commands_encode_exactly();
  reverse_is_encoded_as_direction_zero();
  left_and_right_select_different_motors();
  speeds_below_the_deadband_become_zero();
  speeds_are_clamped_to_the_hardware_range();
  replies_parse();
  implausible_ranges_are_rejected();
  robot_messages_are_replay_safe();
  the_link_talks_to_a_fake_robot();
  connecting_to_nothing_fails_fast();
  a_dead_robot_is_detected();
  return ls::test::summary("elegoo");
}
