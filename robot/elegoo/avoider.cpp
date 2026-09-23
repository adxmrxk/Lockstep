// avoider : drive forward, back off when something is close.
//
// This is the node that shows why the bus is shaped the way it is. It contains
// no networking, no JSON, no notion that an Elegoo car exists. It subscribes to
// a distance and publishes a pair of motor speeds. Swap the bridge for a
// different robot and this file does not change.
//
// It also demonstrates the actual point of the project, on real hardware data:
//
//   --journal <path>   record the session: every range reading as delivered,
//                      every clock read, every decision
//   --replay <path>    re-run those decisions from the journal and print the
//                      output hash
//
// Record a drive, then replay it twice. The hashes match, bit for bit, because
// the journal reproduces not just what the robot SAW but when it saw it and
// what the clock said at the time. That is the thing a ROS 2 bag cannot do, and
// the reason a timing-dependent bug that happens once on the carpet can be made
// to happen again, on demand, at a desk.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "lockstep/core/process.hpp"
#include "lockstep/pubsub.hpp"
#include "lockstep/replay/journal.hpp"
#include "robot_msgs.hpp"

namespace {

// Tuning. Deliberately crude -- the interesting part is reproducibility, not
// the sophistication of the controller.
constexpr float kStopDistanceCm = 25.0f;   // closer than this: retreat
constexpr float kSlowDistanceCm = 50.0f;   // closer than this: ease off
constexpr int kCruiseSpeed = 150;
constexpr int kSlowSpeed = 110;
constexpr int kTurnSpeed = 130;

// THE decision. A pure function of the reading and the clock, which is exactly
// what makes it replayable: give it the same inputs and it must produce the
// same output, every time, forever.
//
// `turn_phase` is deliberately derived from the clock rather than from a
// counter. That is not arbitrary -- it is the kind of hidden timing dependency
// that makes real robot bugs irreproducible, and having one here means the
// replay test is proving something instead of restating that pure functions are
// pure.
RobotDrive decide(const RobotRange& r, std::uint64_t now_ns, std::uint32_t seq) {
  RobotDrive d{};
  d.stamp_ns = now_ns;
  d.seq = seq;
  d.reserved = 0;

  if (r.valid == 0) {
    // No usable reading. Creep rather than stop dead: the sensor misreads
    // often enough that halting on every bad echo would make the car stutter
    // across the floor.
    d.left = kSlowSpeed;
    d.right = kSlowSpeed;
    return d;
  }

  if (r.distance_cm < kStopDistanceCm) {
    // Back and turn. Which way depends on the clock, so two runs that see the
    // same obstacle at different moments will turn differently -- and a replay
    // has to reproduce the clock to reproduce the turn.
    const bool turn_left = ((now_ns / 1000000ull) % 2ull) == 0ull;
    d.left = turn_left ? -kTurnSpeed : kTurnSpeed;
    d.right = turn_left ? kTurnSpeed : -kTurnSpeed;
    return d;
  }

  const int speed = r.distance_cm < kSlowDistanceCm ? kSlowSpeed : kCruiseSpeed;
  d.left = speed;
  d.right = speed;
  return d;
}

struct options {
  std::string bus = "lockstep-robot";
  std::string journal;
  std::string replay;
  long duration_ms = 0;
};

void usage() {
  std::printf(
      "avoider -- obstacle-avoidance node for the Lockstep robot bus\n\n"
      "  --bus <name>      segment to attach to (default lockstep-robot)\n"
      "  --journal <path>  record the session for later replay\n"
      "  --replay <path>   replay a recorded session and print its hash\n"
      "  --duration <ms>   run time; 0 means until the bridge stops\n");
}

// Replay: no bus, no robot, no clock. Every input comes from the journal.
int run_replay(const std::string& path) {
  ls::journal j = ls::journal::open_read(path);
  ls::deterministic_clock clk(ls::run_mode::replay, &j);
  ls::output_hasher out;

  std::uint32_t seq = 0;
  std::uint64_t decisions = 0;
  ls::record rec;
  while (j.next(rec)) {
    if (rec.kind != ls::record_kind::message) continue;
    RobotRange r{};
    std::memcpy(&r, rec.payload, sizeof(r));

    // The clock read happens INSIDE decide() in the recorded run too, so the
    // journal's clock record lands in the same place in the stream.
    const RobotDrive d = decide(r, clk.now_ns(), seq++);
    out.mix_value(d.left);
    out.mix_value(d.right);
    ++decisions;
  }

  std::printf("  [replay] %llu decisions from %llu records\n",
              static_cast<unsigned long long>(decisions),
              static_cast<unsigned long long>(j.records()));
  std::printf("  [replay] output hash %016llx\n",
              static_cast<unsigned long long>(out.value()));
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
    if (a == "--help" || a == "-h") { usage(); return 0; }
    else if (a == "--bus") { const char* v = next(); if (!v) return 1; opt.bus = v; }
    else if (a == "--journal") { const char* v = next(); if (!v) return 1; opt.journal = v; }
    else if (a == "--replay") { const char* v = next(); if (!v) return 1; opt.replay = v; }
    else if (a == "--duration") { const char* v = next(); if (!v) return 1;
                                  opt.duration_ms = std::atol(v); }
    else { std::fprintf(stderr, "unknown option: %s\n", a.c_str()); usage(); return 1; }
  }

  if (!opt.replay.empty()) return run_replay(opt.replay);

  std::printf("avoider\n=======\n");

  ls::bus b;
  try {
    b = ls::bus::open(opt.bus);
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "  cannot attach to bus '%s': %s\n", opt.bus.c_str(), ex.what());
    std::fprintf(stderr, "  start elegoo_bridge first (it creates the bus).\n");
    return 1;
  }

  ls::subscriber<RobotRange> range_sub;
  try {
    range_sub = ls::subscriber<RobotRange>::attach(b, robot_topics::range);
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "  no %s topic yet: %s\n", robot_topics::range, ex.what());
    return 1;
  }
  auto drive_pub = ls::publisher<RobotDrive>::create(b, robot_topics::drive, 32);

  ls::journal jr;
  const bool recording = !opt.journal.empty();
  if (recording) {
    jr = ls::journal::create(opt.journal, 32u << 20);
    std::printf("  recording to %s\n", opt.journal.c_str());
  }
  ls::deterministic_clock clk(recording ? ls::run_mode::record : ls::run_mode::live,
                              recording ? &jr : nullptr);
  ls::output_hasher out;

  std::printf("  %s --> decide --> %s\n\n", robot_topics::range, robot_topics::drive);

  const std::uint64_t t0 = ls::now_ns();
  std::uint32_t seq = 0;
  std::uint64_t decisions = 0, stale = 0;

  for (;;) {
    if (opt.duration_ms > 0 &&
        (ls::now_ns() - t0) / 1000000u >= static_cast<std::uint64_t>(opt.duration_ms)) {
      break;
    }

    ls::sample<RobotRange> s;
    const auto res = range_sub.take(s);
    if (res == ls::read_result::retry) continue;
    if (res != ls::read_result::ok) { ls::sleep_us(2000); continue; }

    RobotRange r{};
    if (!s.copy_out(r)) { ++stale; continue; }  // publisher lapped us mid-read

    // Journal the input BEFORE deciding, so the stream reads: reading, then the
    // clock read that decision made. Replay walks it in the same order.
    if (recording) {
      jr.append(ls::record_kind::message, 0, s.sequence(), s.stamp_ns(), &r, sizeof(r));
    }

    const RobotDrive d = decide(r, clk.now_ns(), seq++);
    out.mix_value(d.left);
    out.mix_value(d.right);
    ++decisions;

    auto ln = drive_pub.loan_message();
    *ln.get() = d;
    drive_pub.publish(ln);
  }

  std::printf("  [done] %llu decisions, %llu stale samples dropped\n",
              static_cast<unsigned long long>(decisions),
              static_cast<unsigned long long>(stale));
  std::printf("  [done] output hash %016llx\n",
              static_cast<unsigned long long>(out.value()));
  if (recording) {
    std::printf("  [done] journal %llu records -- replay it with:\n"
                "         avoider --replay %s\n",
                static_cast<unsigned long long>(jr.records()), opt.journal.c_str());
  }
  return 0;
}
