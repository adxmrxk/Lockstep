// shm/ring.hpp : the broadcast ring every topic publishes through.
//
// A queue is the wrong shape for a robot bus. Perception publishes a frame and
// three nodes want it -- a consuming queue would hand it to whichever asked
// first. So this is a broadcast ring: the publisher writes slots in monotonic
// ticket order, and every subscriber holds its own cursor and reads all of
// them. Falling behind costs you messages, not correctness, and the overrun is
// counted rather than hidden.
//
// THE PROTOCOL (this is the part phase 3 exists to pin down)
//
// Each slot carries one atomic `state` word which is both a sequence number and
// a torn-read detector, in the manner of a seqlock:
//
//     state == 2*ticket      slot is being written for `ticket`
//     state == 2*ticket + 1  slot holds a complete message for `ticket`
//
// Publisher, for ticket t at slot t & mask:
//     1. state.store(2t, release)         -- claim; readers now see it as busy
//     2. write payload_offset, size, pid  -- plain stores, ordered by 1 and 3
//     3. state.store(2t+1, release)       -- commit
//
// Subscriber at cursor c, slot c & mask:
//     1. s1 = state.load(acquire)
//     2. if s1 < 2c+1        -> nothing published for c yet, return empty
//        if s1 > 2c+1        -> we were lapped; jump the cursor forward, count
//                               an overrun, and retry
//     3. copy the slot's fields out
//     4. s2 = state.load(acquire); if s2 != s1 the publisher overwrote the slot
//        underneath us, so the copy is torn: discard it and retry
//
// Step 4 is what makes a lapped reader safe without the publisher ever waiting
// on it. A publisher never blocks, never allocates and never inspects a
// subscriber, which is what keeps its worst case bounded.
//
// ONE PUBLISHER PER TOPIC. This is a hard requirement, not a simplification,
// and it was TLC that established it rather than reasoning or testing.
//
// The obvious generalisation -- hand out tickets with a fetch_add and let
// several publishers share a topic -- is broken, and broken in a way the
// concurrency test in tests/test_ring.cpp did NOT catch. TLC finds it in a
// 16-state trace (docs/tla/Ring.cfg reproduces it):
//
//   p1 reserves ticket 0 and stalls after writing its payload.
//   p2 publishes tickets 1 and 2. Ticket 2 lands on the same slot as 0.
//   p2 claims that slot (state := 4) and writes payload 2.
//   p1 finally commits ticket 0, storing state := 1 over the top.
//
// The slot now reads state == 1, meaning "ticket 0, committed", while holding
// ticket 2's payload. A subscriber at cursor 0 loads 1, copies payload 2,
// re-loads 1, sees no change and accepts. That is a torn read that passes every
// check in the protocol, because a late publisher made the state word go
// BACKWARDS and the whole seqlock argument rests on it only going forwards.
//
// A CAS on the claim does not rescue it: two publishers can still interleave
// their payload writes between one another's claim and commit. The honest fix
// is to forbid the configuration, which costs nothing here -- a sensor node
// owns its topic -- and is required by phase 7 regardless, since two publishers
// racing on one topic produce an interleaving that cannot be replayed
// deterministically. registry::announce enforces it: the second publisher on a
// topic is refused with attach_status::already_published.
//
// With one publisher the state word per slot is monotonically non-decreasing,
// which is exactly the seqlock precondition. TLC checks that configuration
// exhaustively: docs/tla/Ring_spmc.cfg, 65137 distinct states, no violation.
//
// The model checks the ALGORITHM under sequential consistency. It does not
// check the acquire/release barriers -- see docs/tla/README.md.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ls {

struct ring_slot {
  std::atomic<std::uint64_t> state;  // 2t = writing, 2t+1 = committed
  std::atomic<std::uint32_t> refcnt;
  std::uint32_t owner_pid;
  // Both offsets are segment-relative and are assigned ONCE, when the topic is
  // created, then never mutated. Each slot owns its message block and its
  // payload block for the life of the bus, so publishing allocates nothing and
  // nothing ever has to be reclaimed. That is what keeps the publish path
  // bounded, and it is why the seqlock below only has to protect the block's
  // CONTENTS rather than a changing pointer to it.
  std::uint64_t payload_offset;  // the T itself
  std::uint64_t body_offset;     // out-of-line bytes (a frame, a point cloud)
  std::uint64_t body_capacity;

  std::uint64_t payload_size;  // bytes of body actually used by this message
  std::uint64_t stamp_ns;
  std::uint64_t sequence;  // ticket, duplicated for the journal's benefit
};

struct ring_header {
  std::uint64_t capacity;  // always a power of two
  std::uint64_t mask;
  std::atomic<std::uint64_t> write_pos;
  std::atomic<std::uint64_t> overruns;
  ring_slot slots[1];  // capacity entries; the struct is a header, not a value
};

static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "ring state words must be lock-free to cross a process boundary");

// What a subscriber got back from one read attempt.
enum class read_result : std::uint32_t {
  ok = 0,
  empty,     // nothing published at this cursor yet
  overrun,   // the publisher lapped us; the cursor has been moved forward
  retry,     // torn read, the caller should try again
};

// One message, copied out of a slot by value. Small and trivially copyable:
// the payload itself stays in the arena, this is just the descriptor.
struct ring_view {
  std::uint64_t sequence;
  std::uint64_t payload_offset;
  std::uint64_t body_offset;
  std::uint64_t payload_size;
  std::uint64_t stamp_ns;
  std::uint32_t owner_pid;
};

class ring {
 public:
  ring() = default;
  explicit ring(ring_header* hdr) noexcept : hdr_(hdr) {}

  static std::size_t bytes_required(std::size_t capacity) noexcept {
    return sizeof(ring_header) + (capacity - 1) * sizeof(ring_slot);
  }

  // capacity must be a power of two so the cursor can be masked rather than
  // divided; a division on the publish path is not something a deadline-bound
  // node should be paying for.
  static ring construct(void* segment_base, std::size_t offset,
                        std::size_t capacity) noexcept {
    auto* hdr = reinterpret_cast<ring_header*>(static_cast<char*>(segment_base) + offset);
    hdr->capacity = capacity;
    hdr->mask = capacity - 1;
    hdr->write_pos.store(0, std::memory_order_relaxed);
    hdr->overruns.store(0, std::memory_order_relaxed);
    for (std::size_t i = 0; i < capacity; ++i) {
      ring_slot& s = hdr->slots[i];
      s.state.store(0, std::memory_order_relaxed);
      s.refcnt.store(0, std::memory_order_relaxed);
      s.owner_pid = 0;
      s.payload_offset = 0;
      s.body_offset = 0;
      s.body_capacity = 0;
      s.payload_size = 0;
      s.stamp_ns = 0;
      s.sequence = 0;
    }
    std::atomic_thread_fence(std::memory_order_release);
    return ring(hdr);
  }

  bool valid() const noexcept { return hdr_ != nullptr; }
  std::uint64_t capacity() const noexcept { return hdr_->capacity; }
  std::uint64_t write_pos() const noexcept {
    return hdr_->write_pos.load(std::memory_order_acquire);
  }
  std::uint64_t overruns() const noexcept {
    return hdr_->overruns.load(std::memory_order_relaxed);
  }

  // Take the next ticket. Wait-free: one fetch_add, no loop.
  std::uint64_t reserve() noexcept {
    return hdr_->write_pos.fetch_add(1, std::memory_order_acq_rel);
  }

  // Publish a payload at a ticket taken from reserve(). Never blocks and never
  // inspects a subscriber: if a reader is still on this slot it gets a torn
  // read and retries, which is strictly better than stalling the publisher.
  void commit(std::uint64_t ticket, std::uint64_t payload_offset,
              std::uint64_t payload_size, std::uint32_t pid,
              std::uint64_t stamp_ns) noexcept {
    ring_slot& s = hdr_->slots[ticket & hdr_->mask];

    s.state.store(2 * ticket, std::memory_order_release);

    s.payload_offset = payload_offset;
    s.payload_size = payload_size;
    s.owner_pid = pid;
    s.stamp_ns = stamp_ns;
    s.sequence = ticket;

    s.state.store(2 * ticket + 1, std::memory_order_release);
  }

  // Read at `cursor`. On overrun the cursor is advanced to the oldest message
  // still in the ring and read_result::overrun is returned, so the caller can
  // count the loss instead of silently skipping it.
  read_result read(std::uint64_t& cursor, ring_view& out) const noexcept {
    ring_slot& s = hdr_->slots[cursor & hdr_->mask];

    const std::uint64_t s1 = s.state.load(std::memory_order_acquire);
    const std::uint64_t want = 2 * cursor + 1;

    if (s1 < want) return read_result::empty;

    if (s1 > want) {
      // The slot has moved on to a later ticket, so our message is gone. The
      // oldest ticket still present is write_pos - capacity.
      const std::uint64_t wp = hdr_->write_pos.load(std::memory_order_acquire);
      const std::uint64_t oldest = (wp > hdr_->capacity) ? wp - hdr_->capacity : 0;
      cursor = (oldest > cursor) ? oldest : cursor + 1;
      hdr_->overruns.fetch_add(1, std::memory_order_relaxed);
      return read_result::overrun;
    }

    out.sequence = s.sequence;
    out.payload_offset = s.payload_offset;
    out.body_offset = s.body_offset;
    out.payload_size = s.payload_size;
    out.stamp_ns = s.stamp_ns;
    out.owner_pid = s.owner_pid;

    // If the word moved while we copied, the publisher overwrote this slot and
    // what we just read is a mix of two messages.
    const std::uint64_t s2 = s.state.load(std::memory_order_acquire);
    if (s2 != s1) return read_result::retry;

    ++cursor;
    return read_result::ok;
  }

  ring_slot& slot(std::uint64_t i) noexcept { return hdr_->slots[i & hdr_->mask]; }

 private:
  ring_header* hdr_ = nullptr;
};

}  // namespace ls
