// Phase 7: record a run, replay it 1000 times, demand the same bits every time.
//
// The trap this test has to avoid is being vacuous. A replay that only re-feeds
// message bytes into a pure function will of course reproduce -- it would
// reproduce with no replay engine at all. So the workload here deliberately
// depends on a wall-clock read, which is the classic reason a "replayed" ROS 2
// run diverges, and the test asserts BOTH directions:
//
//   * replaying the journal reproduces the output hash exactly, 1000/1000;
//   * re-running the same code LIVE does not, because the clock moved.
//
// The second assertion is what gives the first one meaning.
#include <cstdint>
#include <cstdio>
#include <string>
#include <unistd.h>
#include <vector>

#include "lockstep/pubsub.hpp"
#include "lockstep/replay/journal.hpp"
#include "support/check.hpp"
#include "support/demo_msgs.hpp"

namespace {

constexpr int kMessages = 200;
constexpr int kReplays = 1000;

std::string tmp_path(const char* tag) {
  return std::string("/tmp/lockstep-") + tag + "-" + std::to_string(::getpid()) + ".jrnl";
}

std::string unique_name(const char* tag) {
  return std::string("lockstep-replay-") + tag + "-" + std::to_string(::getpid());
}

// The node under test. Its output depends on the message AND on what the clock
// said, so it is only reproducible if both are reproduced.
void handle(const ImuSample& m, ls::deterministic_clock& clk, ls::output_hasher& out,
            std::uint64_t& accum) {
  const std::uint64_t t = clk.now_ns();
  accum = accum * 1315423911ull + m.stamp_ns + t;
  out.mix_value(accum);
  out.mix_value(m.ax);
  // A data-dependent branch, so divergence changes control flow and not just
  // arithmetic -- this is what makes a real bug show up or not show up.
  if ((accum & 0xff) < 64) out.mix_value(m.gz);
}

struct run_result {
  std::uint64_t hash = 0;
  std::uint64_t items = 0;
  std::uint64_t delivered = 0;
  std::uint64_t clock_reads = 0;
};

// Record: drive the real bus, journal every delivered message and every clock
// read, and hash the outputs.
run_result record_run(const std::string& journal_path) {
  const std::string name = unique_name("rec");
  ls::segment::unlink(name);
  ls::bus b = ls::bus::create(name, {{64, 512}, {4096, 16}, {65536, 4}});

  auto pub = ls::publisher<ImuSample>::create(b, "imu", 256);
  auto sub = ls::subscriber<ImuSample>::attach(b, "imu");

  ls::journal j = ls::journal::create(journal_path, 8u << 20);
  ls::deterministic_clock clk(ls::run_mode::record, &j);
  ls::output_hasher out;
  std::uint64_t accum = 1;

  run_result r;
  for (int i = 0; i < kMessages; ++i) {
    auto ln = pub.loan_message();
    ln->stamp_ns = static_cast<std::uint64_t>(i) * 37u + 11u;
    ln->ax = static_cast<float>(i) * 0.25f;
    ln->gz = static_cast<float>(-i);
    pub.publish(ln);

    ls::sample<ImuSample> s;
    for (;;) {
      const auto rr = sub.take(s);
      if (rr == ls::read_result::retry) continue;
      if (rr != ls::read_result::ok) break;

      ImuSample copy{};
      if (!s.copy_out(copy)) break;

      // Journal the message AS DELIVERED, before the callback runs, so the
      // stream reads message, then the clock read that callback made.
      j.append(ls::record_kind::message, 0, s.sequence(), s.stamp_ns(), &copy,
               sizeof(copy));
      handle(copy, clk, out, accum);
      ++r.delivered;
    }
  }

  r.hash = out.value();
  r.items = out.items();
  r.clock_reads = clk.reads();
  return r;
}

// Replay: the same handler, fed entirely from the journal. No bus, no clock.
run_result replay_run(const std::string& journal_path) {
  ls::journal j = ls::journal::open_read(journal_path);
  ls::deterministic_clock clk(ls::run_mode::replay, &j);
  ls::output_hasher out;
  std::uint64_t accum = 1;

  run_result r;
  ls::record rec;
  while (j.next(rec)) {
    if (rec.kind != ls::record_kind::message) continue;
    ImuSample m{};
    std::memcpy(&m, rec.payload, sizeof(m));
    // handle() calls clk.now_ns(), which consumes the clock record that
    // immediately follows this message in the stream.
    handle(m, clk, out, accum);
    ++r.delivered;
  }
  r.hash = out.value();
  r.items = out.items();
  r.clock_reads = clk.reads();
  return r;
}

// The same handler again, but reading the real clock: the control case.
run_result live_rerun(const std::string& journal_path) {
  ls::journal j = ls::journal::open_read(journal_path);
  ls::deterministic_clock clk(ls::run_mode::live, nullptr);
  ls::output_hasher out;
  std::uint64_t accum = 1;

  run_result r;
  ls::record rec;
  while (j.next(rec)) {
    if (rec.kind != ls::record_kind::message) continue;
    ImuSample m{};
    std::memcpy(&m, rec.payload, sizeof(m));
    handle(m, clk, out, accum);
    ++r.delivered;
  }
  r.hash = out.value();
  r.items = out.items();
  return r;
}

void a_journal_round_trips() {
  const std::string path = tmp_path("rt");
  {
    ls::journal j = ls::journal::create(path, 64 * 1024);
    const std::uint8_t payload[4] = {1, 2, 3, 4};
    LS_CHECK(j.append(ls::record_kind::message, 7, 42, 99, payload, 4));
    LS_CHECK(j.append(ls::record_kind::clock, 0, 1, 123456, nullptr, 0));
    LS_CHECK_EQ(j.records(), 2u);
  }

  ls::journal j = ls::journal::open_read(path);
  LS_CHECK_EQ(j.records(), 2u);

  ls::record r;
  LS_CHECK(j.next(r));
  LS_CHECK(r.kind == ls::record_kind::message);
  LS_CHECK_EQ(r.callback, 7u);
  LS_CHECK_EQ(r.sequence, 42u);
  LS_CHECK_EQ(r.bytes, 4u);
  LS_CHECK_EQ(r.payload[2], 3u);

  LS_CHECK(j.next(r));
  LS_CHECK(r.kind == ls::record_kind::clock);
  LS_CHECK_EQ(r.value, 123456u);

  LS_CHECK(!j.next(r));
  ::unlink(path.c_str());
}

void a_full_journal_refuses_rather_than_overruns() {
  const std::string path = tmp_path("full");
  ls::journal j = ls::journal::create(path, 4096);
  std::uint8_t blob[256] = {};
  int written = 0;
  while (j.append(ls::record_kind::message, 0, 0, 0, blob, sizeof(blob))) ++written;
  LS_CHECK(written > 0);
  LS_CHECK(!j.append(ls::record_kind::message, 0, 0, 0, blob, sizeof(blob)));
  LS_CHECK(j.used() <= 4096u);
  ::unlink(path.c_str());
}

// THE test.
void a_thousand_replays_all_match() {
  const std::string path = tmp_path("run");

  const run_result rec = record_run(path);
  LS_CHECK_EQ(rec.delivered, static_cast<std::uint64_t>(kMessages));
  LS_CHECK(rec.clock_reads > 0);
  LS_CHECK(rec.items > 0);

  int matches = 0;
  int mismatches = 0;
  for (int i = 0; i < kReplays; ++i) {
    const run_result rp = replay_run(path);
    if (rp.hash == rec.hash && rp.delivered == rec.delivered &&
        rp.items == rec.items) {
      ++matches;
    } else {
      ++mismatches;
      if (mismatches == 1) {
        std::fprintf(stderr,
                     "  first mismatch at replay %d: hash %llx vs %llx, "
                     "delivered %llu vs %llu\n",
                     i, (unsigned long long)rp.hash, (unsigned long long)rec.hash,
                     (unsigned long long)rp.delivered,
                     (unsigned long long)rec.delivered);
      }
    }
  }

  // The control: same code, same inputs, real clock. If THIS matched, the
  // workload would not actually depend on the clock and the 1000 above would
  // prove nothing.
  const run_result live = live_rerun(path);

  std::fprintf(stderr,
               "  [replay] recorded %llu msgs, %llu clock reads, hash %016llx\n"
               "  [replay] %d/%d replays matched, %d diverged\n"
               "  [replay] live re-run hash %016llx (%s)\n",
               (unsigned long long)rec.delivered,
               (unsigned long long)rec.clock_reads,
               (unsigned long long)rec.hash, matches, kReplays, mismatches,
               (unsigned long long)live.hash,
               live.hash == rec.hash ? "MATCHED - test is vacuous!" : "differs, as it must");

  LS_CHECK_EQ(matches, kReplays);
  LS_CHECK_EQ(mismatches, 0);

  // Anti-vacuity: the output genuinely depends on the clock.
  LS_CHECK(live.hash != rec.hash);
  LS_CHECK_EQ(live.delivered, rec.delivered);  // same inputs, different result

  ::unlink(path.c_str());
}

// Two recordings of the same journal must hash identically; a journal with one
// byte changed must not.
void the_journal_hash_detects_a_changed_byte() {
  const std::string path = tmp_path("hash");
  std::uint64_t original = 0;
  {
    ls::journal j = ls::journal::create(path, 64 * 1024);
    std::uint8_t blob[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    for (int i = 0; i < 8; ++i)
      j.append(ls::record_kind::message, 0, static_cast<std::uint64_t>(i), 0, blob, 16);
    original = j.content_hash();
  }
  {
    ls::journal j = ls::journal::open_read(path);
    LS_CHECK_EQ(j.content_hash(), original);
  }
  {
    // Flip one payload byte.
    ls::journal j = ls::journal::create(path + ".v2", 64 * 1024);
    std::uint8_t blob[16] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
    for (int i = 0; i < 8; ++i) {
      if (i == 3) blob[5] = 99;
      j.append(ls::record_kind::message, 0, static_cast<std::uint64_t>(i), 0, blob, 16);
      if (i == 3) blob[5] = 6;
    }
    LS_CHECK(j.content_hash() != original);
  }
  ::unlink(path.c_str());
  ::unlink((path + ".v2").c_str());
}

}  // namespace

int main() {
  a_journal_round_trips();
  a_full_journal_refuses_rather_than_overruns();
  the_journal_hash_detects_a_changed_byte();
  a_thousand_replays_all_match();
  return ls::test::summary("replay");
}
