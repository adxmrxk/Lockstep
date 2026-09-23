// The broadcast ring protocol.
//
// The invariant that matters most here is not "no messages are lost" -- losing
// messages to a slow reader is the design -- it is that a message a subscriber
// DOES accept is never a mix of two publishers' writes. Every test below that
// touches concurrency checks internal consistency of the returned descriptor,
// because a torn read is the failure this protocol exists to prevent.
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "lockstep/shm/ring.hpp"
#include "lockstep/core/process.hpp"
#include "lockstep/shm/segment.hpp"
#include "support/check.hpp"

namespace {

std::string unique_name(const char* tag) {
  return std::string("lockstep-ring-") + tag + "-" + std::to_string(ls::self_pid());
}

// Fields are derived from the sequence number, so any inconsistency between
// them proves the reader saw two different messages inside one descriptor.
constexpr std::uint64_t stamp_for(std::uint64_t seq) { return seq * 1000 + 7; }
constexpr std::uint64_t offset_for(std::uint64_t seq) { return 4096 + seq * 64; }
constexpr std::uint64_t size_for(std::uint64_t seq) { return 16 + (seq % 48); }

bool descriptor_is_self_consistent(const ls::ring_view& v) {
  return v.stamp_ns == stamp_for(v.sequence) &&
         v.payload_offset == offset_for(v.sequence) &&
         v.payload_size == size_for(v.sequence);
}

struct fixture {
  ls::segment seg;
  ls::ring r;

  fixture(std::size_t capacity, const char* tag) {
    const std::string name = unique_name(tag);
    ls::segment::unlink(name);
    seg = ls::segment::create(name, 4096 + ls::ring::bytes_required(capacity));
    r = ls::ring::construct(seg.base(), 4096, capacity);
  }

  void publish(std::uint64_t n) {
    for (std::uint64_t i = 0; i < n; ++i) {
      const std::uint64_t t = r.reserve();
      r.commit(t, offset_for(t), size_for(t), 1234, stamp_for(t));
    }
  }
};

void an_empty_ring_reads_empty() {
  fixture f(8, "empty");
  std::uint64_t cursor = 0;
  ls::ring_view v{};
  LS_CHECK(f.r.read(cursor, v) == ls::read_result::empty);
  LS_CHECK_EQ(cursor, 0u);  // an empty read must not move the cursor
}

void published_messages_come_back_in_order() {
  fixture f(8, "order");
  f.publish(5);

  std::uint64_t cursor = 0;
  for (std::uint64_t i = 0; i < 5; ++i) {
    ls::ring_view v{};
    LS_CHECK(f.r.read(cursor, v) == ls::read_result::ok);
    LS_CHECK_EQ(v.sequence, i);
    LS_CHECK(descriptor_is_self_consistent(v));
  }
  ls::ring_view v{};
  LS_CHECK(f.r.read(cursor, v) == ls::read_result::empty);
  LS_CHECK_EQ(f.r.overruns(), 0u);
}

// Two subscribers on one topic each get every message: this is a broadcast,
// not a queue, so one reading does not consume it from the other.
void every_subscriber_sees_every_message() {
  fixture f(8, "broadcast");
  f.publish(6);

  for (int sub = 0; sub < 2; ++sub) {
    std::uint64_t cursor = 0;
    std::uint64_t got = 0;
    for (;;) {
      ls::ring_view v{};
      const auto rr = f.r.read(cursor, v);
      if (rr == ls::read_result::empty) break;
      if (rr == ls::read_result::ok) {
        LS_CHECK_EQ(v.sequence, got);
        ++got;
      }
    }
    LS_CHECK_EQ(got, 6u);
  }
}

// A reader that falls more than `capacity` behind loses messages, and must be
// told so rather than silently resyncing.
void a_lapped_reader_is_told_it_was_lapped() {
  fixture f(4, "overrun");
  std::uint64_t cursor = 0;

  f.publish(2);
  ls::ring_view v{};
  LS_CHECK(f.r.read(cursor, v) == ls::read_result::ok);
  LS_CHECK_EQ(v.sequence, 0u);

  // Publish well past the reader's cursor, lapping the 4-slot ring twice.
  f.publish(10);

  const auto rr = f.r.read(cursor, v);
  LS_CHECK(rr == ls::read_result::overrun);
  LS_CHECK_EQ(f.r.overruns(), 1u);

  // The cursor must have been moved to the oldest message still present, which
  // is write_pos - capacity = 12 - 4 = 8.
  LS_CHECK_EQ(cursor, 8u);

  LS_CHECK(f.r.read(cursor, v) == ls::read_result::ok);
  LS_CHECK_EQ(v.sequence, 8u);
  LS_CHECK(descriptor_is_self_consistent(v));
}

void reading_past_the_end_stays_empty() {
  fixture f(8, "past_end");
  f.publish(3);
  std::uint64_t cursor = 3;
  ls::ring_view v{};
  LS_CHECK(f.r.read(cursor, v) == ls::read_result::empty);
  LS_CHECK_EQ(cursor, 3u);
}

// The real test. ONE publisher -- the configuration the registry enforces and
// TLC verified -- hammering a deliberately small ring while subscribers chase
// it, so overruns and torn reads are both frequent. Every descriptor a
// subscriber accepts must still be internally consistent.
//
// This test passed with three publishers too, which is exactly why it is not
// the evidence that matters: TLC found a torn read in the three-publisher case
// that this test never produced. See docs/tla/Ring.cfg. Testing samples the
// interleavings the scheduler happens to pick; the model checker enumerates
// them.
void concurrent_publish_never_hands_a_reader_a_torn_message() {
  constexpr int kPublishers = 1;
  constexpr int kSubscribers = 3;
  constexpr std::uint64_t kPerPublisher = 60000;

  fixture f(16, "concurrent");  // small on purpose: force laps

  std::atomic<bool> done{false};
  std::atomic<std::uint64_t> torn{0};
  std::atomic<std::uint64_t> accepted{0};
  std::atomic<std::uint64_t> out_of_order{0};

  std::vector<std::thread> pubs;
  for (int p = 0; p < kPublishers; ++p) {
    pubs.emplace_back([&] {
      for (std::uint64_t i = 0; i < kPerPublisher; ++i) {
        const std::uint64_t t = f.r.reserve();
        f.r.commit(t, offset_for(t), size_for(t), 7, stamp_for(t));
      }
    });
  }

  std::vector<std::thread> subs;
  for (int s = 0; s < kSubscribers; ++s) {
    subs.emplace_back([&] {
      std::uint64_t cursor = 0;
      std::uint64_t last = 0;
      bool have_last = false;
      while (!done.load(std::memory_order_acquire)) {
        ls::ring_view v{};
        const auto rr = f.r.read(cursor, v);
        if (rr == ls::read_result::ok) {
          if (!descriptor_is_self_consistent(v))
            torn.fetch_add(1, std::memory_order_relaxed);
          if (have_last && v.sequence <= last)
            out_of_order.fetch_add(1, std::memory_order_relaxed);
          last = v.sequence;
          have_last = true;
          accepted.fetch_add(1, std::memory_order_relaxed);
        }
      }
    });
  }

  for (auto& t : pubs) t.join();
  done.store(true, std::memory_order_release);
  for (auto& t : subs) t.join();

  LS_CHECK_EQ(f.r.write_pos(), kPublishers * kPerPublisher);
  LS_CHECK(accepted.load() > 0);       // the subscribers really ran
  LS_CHECK(f.r.overruns() > 0);        // and really were lapped, so retry/overrun
                                       // paths were exercised rather than skipped
  LS_CHECK_EQ(torn.load(), 0u);        // the invariant
  LS_CHECK_EQ(out_of_order.load(), 0u);  // a cursor never goes backwards
}

}  // namespace

int main() {
  an_empty_ring_reads_empty();
  published_messages_come_back_in_order();
  every_subscriber_sees_every_message();
  a_lapped_reader_is_told_it_was_lapped();
  reading_past_the_end_stays_empty();
  concurrent_publish_never_hands_a_reader_a_torn_message();
  return ls::test::summary("ring");
}
