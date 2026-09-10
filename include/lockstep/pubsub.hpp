// pubsub.hpp : loan / publish / subscribe -- the API a node actually uses.
//
//   auto pub = ls::publisher<ImuSample>::create(bus, "imu/raw", 16);
//   auto ln  = pub.loan();          // a writable ImuSample, already in shm
//   ln->stamp_ns = now();           // written ONCE, in its final location
//   pub.publish(ln);                // one release store; no copy
//
//   auto sub = ls::subscriber<ImuSample>::attach(bus, "imu/raw");
//   ls::sample<ImuSample> s;
//   while (sub.take(s) == ls::read_result::ok) use(*s);
//
// ZERO COPY, meant literally. loan() hands back a pointer into the shared
// segment. The publisher's field writes are the only writes the message ever
// receives; publish() stores one word. The subscriber reads through a pointer
// into the same bytes. Nothing is serialised, and the message body is never
// memcpy'd at any point on the path -- tests/test_pubsub.cpp asserts that the
// subscriber's pointer and the publisher's resolve to the same segment offset.
//
// NO ALLOCATION ON THE PUBLISH PATH. Every ring slot is given its message block
// and its body block once, when the topic is created, and keeps them for the
// life of the bus. So publishing never calls the arena, never reclaims, and has
// no failure mode that depends on how much memory is left. This is also what
// phase 6 needs: a deadline-bound node cannot afford an allocator on its hot
// path.
//
// THE READER'S CONTRACT, which is the part to actually read.
//
// This is a lossy broadcast bus. A subscriber holding a sample is holding a
// pointer into a slot the publisher will eventually reuse -- after `capacity`
// further publications. take() validates the slot before handing it over, but
// that validation is a snapshot. A subscriber that dawdles can have the bytes
// change underneath it.
//
// So a sample is valid until you next call take(), and if you are doing
// anything non-trivial with it you must confirm you actually won the race:
//
//   if (sub.take(s) == ls::read_result::ok) {
//     process(*s);
//     if (!s.still_valid()) discard();   // the publisher lapped us mid-read
//   }
//
// Or call s.copy_out(dst), which copies and then validates, and hands you back
// a message that is yours. That costs a copy, which is the honest price of not
// wanting to think about it.
//
// The refcnt in each slot is maintained but not yet enforced by the publisher:
// making the publisher skip a referenced slot is orphan reclamation, which is
// phase 5.
#pragma once

#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>

#include "lockstep/core/clock.hpp"
#include "lockstep/shm/bus.hpp"
#include "lockstep/shm/ring.hpp"

#if !defined(_WIN32)
#include <unistd.h>
#endif

namespace ls {

// ---------------------------------------------------------------------------
// loan: a writable message sitting in its final location in shared memory.
// Move-only, so it cannot be published twice by accident.
// ---------------------------------------------------------------------------
template <class T>
class loan {
 public:
  loan() = default;
  loan(T* msg, std::uint8_t* body, std::size_t body_capacity, std::uint64_t ticket) noexcept
      : msg_(msg), body_(body), body_capacity_(body_capacity), ticket_(ticket) {}

  loan(const loan&) = delete;
  loan& operator=(const loan&) = delete;
  loan(loan&& o) noexcept { swap(o); }
  loan& operator=(loan&& o) noexcept {
    loan tmp(std::move(o));
    swap(tmp);
    return *this;
  }

  bool valid() const noexcept { return msg_ != nullptr; }
  T* get() const noexcept { return msg_; }
  T* operator->() const noexcept { return msg_; }
  T& operator*() const noexcept { return *msg_; }

  // Out-of-line storage for a large payload, already inside the segment.
  std::uint8_t* body() const noexcept { return body_; }
  std::size_t body_capacity() const noexcept { return body_capacity_; }
  std::uint64_t ticket() const noexcept { return ticket_; }

  void set_body_size(std::size_t n) noexcept { body_size_ = n; }
  std::size_t body_size() const noexcept { return body_size_; }

 private:
  void swap(loan& o) noexcept {
    std::swap(msg_, o.msg_);
    std::swap(body_, o.body_);
    std::swap(body_capacity_, o.body_capacity_);
    std::swap(body_size_, o.body_size_);
    std::swap(ticket_, o.ticket_);
  }

  T* msg_ = nullptr;
  std::uint8_t* body_ = nullptr;
  std::size_t body_capacity_ = 0;
  std::size_t body_size_ = 0;
  std::uint64_t ticket_ = 0;
};

// ---------------------------------------------------------------------------
// sample: a const view of a published message, still in shared memory.
// ---------------------------------------------------------------------------
template <class T>
class sample {
 public:
  sample() = default;

  bool valid() const noexcept { return msg_ != nullptr; }
  const T* get() const noexcept { return msg_; }
  const T* operator->() const noexcept { return msg_; }
  const T& operator*() const noexcept { return *msg_; }

  const std::uint8_t* body() const noexcept { return body_; }
  std::size_t body_size() const noexcept { return body_size_; }
  std::uint64_t sequence() const noexcept { return sequence_; }
  std::uint64_t stamp_ns() const noexcept { return stamp_ns_; }

  // Did the publisher overwrite this slot while we were looking at it? Cheap:
  // one acquire load of the slot's state word.
  bool still_valid() const noexcept {
    if (slot_ == nullptr) return false;
    return slot_->state.load(std::memory_order_acquire) == guard_;
  }

  // Copy the message out and confirm the copy is coherent. Returns false if the
  // publisher lapped us mid-copy, in which case `dst` is garbage and should be
  // discarded.
  bool copy_out(T& dst) const noexcept {
    if (msg_ == nullptr) return false;
    std::memcpy(static_cast<void*>(&dst), static_cast<const void*>(msg_), sizeof(T));
    return still_valid();
  }

 private:
  template <class>
  friend class subscriber;

  const T* msg_ = nullptr;
  const std::uint8_t* body_ = nullptr;
  std::size_t body_size_ = 0;
  std::uint64_t sequence_ = 0;
  std::uint64_t stamp_ns_ = 0;
  const ring_slot* slot_ = nullptr;
  std::uint64_t guard_ = 0;
};

// ---------------------------------------------------------------------------
// publisher
// ---------------------------------------------------------------------------
template <class T>
class publisher {
 public:
  static_assert(message_traits<T>::declared,
                "publisher<T> needs a LOCKSTEP_MESSAGE type");

  // capacity must be a power of two. body_bytes is the out-of-line payload each
  // slot gets; pass 0 for messages that carry everything inline.
  static publisher create(bus& b, std::string_view topic, std::size_t capacity,
                          std::size_t body_bytes = 0) {
    if ((capacity & (capacity - 1)) != 0 || capacity == 0)
      throw std::invalid_argument("lockstep: ring capacity must be a power of two");

    publisher p;
    p.bus_ = &b;

    const std::uint32_t pid = current_pid();
    const attach_status st =
        b.topics().template announce<T>(topic, &p.topic_, /*exclusive=*/true, pid);
    if (st != attach_status::ok)
      throw std::runtime_error(std::string("lockstep: cannot publish ") +
                               std::string(topic) + ": " + to_string(st));

    // If the topic already has a ring, adopt it; otherwise build one.
    const std::uint64_t existing = p.topic_->ring_offset.load(std::memory_order_acquire);
    if (existing != 0) {
      p.ring_ = ring(b.seg().template at<ring_header>(existing));
    } else {
      const std::uint64_t ring_off =
          b.allocator().allocate(ring::bytes_required(capacity));
      if (ring_off == 0)
        throw std::runtime_error("lockstep: arena cannot fit the ring for " +
                                 std::string(topic));
      p.ring_ = ring::construct(b.seg().base(), ring_off, capacity);

      // Give every slot its permanent message and body blocks now, so that
      // publishing later never touches the allocator.
      for (std::uint64_t i = 0; i < capacity; ++i) {
        ring_slot& s = p.ring_.slot(i);
        const std::uint64_t mo = b.allocator().allocate(sizeof(T));
        if (mo == 0)
          throw std::runtime_error("lockstep: arena cannot fit slot storage for " +
                                   std::string(topic));
        s.payload_offset = mo;

        // Zero the whole block. message_traits<T>::has_padding tells us whether
        // T has indeterminate bytes; a replay hash is taken over the entire
        // slot, so those have to start at a known value.
        std::memset(b.seg().template at<void>(mo), 0, sizeof(T));

        if (body_bytes > 0) {
          const std::uint64_t bo = b.allocator().allocate(body_bytes);
          if (bo == 0)
            throw std::runtime_error("lockstep: arena cannot fit body storage for " +
                                     std::string(topic));
          s.body_offset = bo;
          s.body_capacity = body_bytes;
          std::memset(b.seg().template at<void>(bo), 0, body_bytes);
        }
      }
      p.topic_->ring_offset.store(ring_off, std::memory_order_release);
    }

    p.topic_->heartbeat_ns.store(now_ns(), std::memory_order_relaxed);
    return p;
  }

  // Take the next slot and hand back a writable message in shared memory.
  // Allocation-free and wait-free: one fetch_add on the ring's write position.
  loan<T> loan_message() noexcept {
    const std::uint64_t ticket = ring_.reserve();
    ring_slot& s = ring_.slot(ticket);

    // Claim the slot before touching its contents, so a subscriber reading it
    // right now sees "busy" rather than a half-written message.
    s.state.store(2 * ticket, std::memory_order_release);
    s.flags.store(0, std::memory_order_relaxed);

    auto* msg = bus_->seg().template at<T>(s.payload_offset);
    auto* body = s.body_offset ? bus_->seg().template at<std::uint8_t>(s.body_offset)
                               : nullptr;

    // A recycled slot still holds the previous message's bytes. Replay hashes
    // cover the whole slot, so it has to start from a known state.
    std::memset(static_cast<void*>(msg), 0, sizeof(T));

    return loan<T>(msg, body, s.body_capacity, ticket);
  }

  // Commit. One release store; the message is already where it belongs.
  bool publish(loan<T>& ln) noexcept {
    if (!ln.valid()) return false;
    const std::uint64_t ticket = ln.ticket();
    ring_slot& s = ring_.slot(ticket);

    s.payload_size = ln.body_size();
    s.owner_pid = current_pid();
    s.stamp_ns = now_ns();
    s.sequence = ticket;

    s.state.store(2 * ticket + 1, std::memory_order_release);
    topic_->heartbeat_ns.store(s.stamp_ns, std::memory_order_relaxed);
    ++published_;
    return true;
  }

  topic_entry* topic() const noexcept { return topic_; }
  ring& get_ring() noexcept { return ring_; }
  std::uint64_t published() const noexcept { return published_; }

  static std::uint32_t current_pid() noexcept {
#if defined(_WIN32)
    return 0;
#else
    return static_cast<std::uint32_t>(::getpid());
#endif
  }

 private:
  bus* bus_ = nullptr;
  topic_entry* topic_ = nullptr;
  ring ring_;
  std::uint64_t published_ = 0;
};

// ---------------------------------------------------------------------------
// subscriber
// ---------------------------------------------------------------------------
template <class T>
class subscriber {
 public:
  static_assert(message_traits<T>::declared,
                "subscriber<T> needs a LOCKSTEP_MESSAGE type");

  static subscriber attach(bus& b, std::string_view topic) {
    subscriber s;
    s.bus_ = &b;

    const attach_status st = b.topics().template attach<T>(topic, &s.topic_);
    if (st != attach_status::ok)
      throw std::runtime_error(std::string("lockstep: cannot subscribe to ") +
                               std::string(topic) + ": " + to_string(st));

    const std::uint64_t ro = s.topic_->ring_offset.load(std::memory_order_acquire);
    if (ro == 0)
      throw std::runtime_error("lockstep: topic " + std::string(topic) +
                               " has no ring yet; the publisher has not started");
    s.ring_ = ring(b.seg().template at<ring_header>(ro));

    // Start at the oldest message still resident rather than at 0, so a late
    // subscriber does not immediately report an overrun for history it was
    // never going to see.
    const std::uint64_t wp = s.ring_.write_pos();
    s.cursor_ = (wp > s.ring_.capacity()) ? wp - s.ring_.capacity() : 0;
    return s;
  }

  // Read the next message. On ok, `out` points into shared memory -- see the
  // reader's contract at the top of this file.
  read_result take(sample<T>& out) noexcept {
    ring_view v{};
    const read_result rr = ring_.read(cursor_, v);
    if (rr != read_result::ok) {
      if (rr == read_result::overrun) ++overruns_;
      return rr;
    }

    const ring_slot& s = ring_.slot(v.sequence);
    out.msg_ = bus_->seg().template at<const T>(v.payload_offset);
    out.body_ = v.body_offset
                    ? bus_->seg().template at<const std::uint8_t>(v.body_offset)
                    : nullptr;
    out.body_size_ = v.payload_size;
    out.sequence_ = v.sequence;
    out.stamp_ns_ = v.stamp_ns;
    out.slot_ = &s;
    out.guard_ = 2 * v.sequence + 1;
    ++received_;
    return read_result::ok;
  }

  std::uint64_t cursor() const noexcept { return cursor_; }
  std::uint64_t received() const noexcept { return received_; }
  std::uint64_t overruns() const noexcept { return overruns_; }
  topic_entry* topic() const noexcept { return topic_; }

 private:
  bus* bus_ = nullptr;
  topic_entry* topic_ = nullptr;
  ring ring_;
  std::uint64_t cursor_ = 0;
  std::uint64_t received_ = 0;
  std::uint64_t overruns_ = 0;
};

}  // namespace ls
