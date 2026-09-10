// shm/registry.hpp : the topic table at the head of the segment.
//
// This is where phase 1 stops being a type-system exercise and starts earning
// its keep. Every topic records the layout hash of the message type it carries,
// and a process attaching to a topic must present a matching hash. A node built
// against a header somebody has since reordered is refused at connect time
// instead of quietly misreading every field for the rest of the run.
//
// Slots are claimed with a CAS on a three-state word (empty -> claiming ->
// ready), so two processes announcing the same topic at the same moment produce
// one winner and one attacher, never two half-written entries. Names are
// compared byte-for-byte out of a fixed char array; no std::string crosses the
// boundary, for the usual reason.
#pragma once

#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "lockstep/core/message.hpp"

#if !defined(_WIN32)
#include <sched.h>
#include <unistd.h>
#endif

namespace ls {

inline constexpr std::size_t max_topics = 64;
inline constexpr std::size_t max_topic_name = 64;

enum class topic_state : std::uint32_t { empty = 0, claiming = 1, ready = 2 };

// Why an attach was refused. Returned rather than thrown: this is the one
// failure a node is expected to handle, and it happens at setup, not on the
// hot path.
enum class attach_status : std::uint32_t {
  ok = 0,
  not_found,
  layout_mismatch,
  size_mismatch,
  registry_full,
  name_too_long,
  already_published,
};

inline const char* to_string(attach_status s) noexcept {
  switch (s) {
    case attach_status::ok: return "ok";
    case attach_status::not_found: return "topic not found";
    case attach_status::layout_mismatch: return "layout hash mismatch";
    case attach_status::size_mismatch: return "message size mismatch";
    case attach_status::registry_full: return "registry full";
    case attach_status::name_too_long: return "topic name too long";
    case attach_status::already_published: return "topic already has a publisher";
  }
  return "unknown";
}

struct topic_entry {
  std::atomic<std::uint32_t> state;
  std::uint32_t name_len;
  char name[max_topic_name];

  // The phase 1 handshake, carried across the boundary.
  std::uint64_t layout_hash;
  std::uint64_t message_size;
  std::uint32_t message_align;
  std::uint32_t message_has_padding;

  // Liveness. Phase 5 turns these into orphan reclamation; for now they are
  // recorded and read back, nothing more.
  std::atomic<std::uint32_t> publisher_pid;
  std::atomic<std::uint64_t> heartbeat_ns;

  // Filled in by phase 4 when the ring lands here.
  std::atomic<std::uint64_t> ring_offset;

  std::string_view topic_name() const noexcept {
    return std::string_view(name, name_len);
  }
};

struct registry_header {
  std::uint32_t capacity;
  std::uint32_t reserved;
  std::atomic<std::uint32_t> used;

  // Serialises topic CREATION only, and holds the creating pid rather than a
  // bare flag so a process that dies mid-create can be detected and displaced.
  //
  // Why a lock at all, when everything else here is lock-free: without one, two
  // processes announcing the same new topic can both miss it in find() and then
  // each claim a DIFFERENT slot for the same name. The result is two entries,
  // two rings, and a publisher and a subscriber that agree on the topic name
  // while pointing at different memory. A double-checked claim does not fix it
  // either, because the loser cannot distinguish "no such topic" from "a topic
  // whose name has been reserved but not yet written".
  //
  // This costs nothing that matters: creation happens at node startup, not on
  // the publish path, which stays lock-free and allocation-free.
  std::atomic<std::uint32_t> create_lock;

  topic_entry topics[max_topics];
};

static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "registry atomics must be lock-free to be shared across processes");

namespace detail {

inline bool registry_pid_alive(std::uint32_t pid) noexcept {
#if defined(_WIN32)
  return pid != 0;
#else
  if (pid == 0) return false;
  return ::kill(static_cast<int>(pid), 0) == 0 || errno == EPERM;
#endif
}

inline std::uint32_t registry_self_pid() noexcept {
#if defined(_WIN32)
  return 1;
#else
  return static_cast<std::uint32_t>(::getpid());
#endif
}

// Held across the topic-creation path. Steals the lock from a holder that is no
// longer alive, so a node killed inside announce() cannot wedge the bus.
class create_guard {
 public:
  explicit create_guard(registry_header& h) noexcept : hdr_(&h) {
    const std::uint32_t self = registry_self_pid();
    for (;;) {
      std::uint32_t expected = 0;
      if (hdr_->create_lock.compare_exchange_weak(expected, self,
                                                  std::memory_order_acq_rel,
                                                  std::memory_order_acquire)) {
        return;
      }
      if (expected != 0 && expected != self && !registry_pid_alive(expected)) {
        hdr_->create_lock.compare_exchange_strong(expected, self,
                                                  std::memory_order_acq_rel,
                                                  std::memory_order_acquire);
        continue;
      }
#if !defined(_WIN32)
      ::sched_yield();
#endif
    }
  }

  ~create_guard() { hdr_->create_lock.store(0, std::memory_order_release); }

  create_guard(const create_guard&) = delete;
  create_guard& operator=(const create_guard&) = delete;

 private:
  registry_header* hdr_;
};

}  // namespace detail

// A view over a registry_header living inside a mapped segment.
class registry {
 public:
  registry() = default;
  explicit registry(registry_header* hdr) noexcept : hdr_(hdr) {}

  static std::size_t bytes_required() noexcept { return sizeof(registry_header); }

  // Zero the table. Called once by the segment's creator.
  static registry construct(void* segment_base, std::size_t offset) noexcept {
    auto* hdr = reinterpret_cast<registry_header*>(
        static_cast<char*>(segment_base) + offset);
    std::memset(static_cast<void*>(hdr), 0, sizeof(registry_header));
    hdr->capacity = static_cast<std::uint32_t>(max_topics);
    hdr->used.store(0, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    return registry(hdr);
  }

  bool valid() const noexcept { return hdr_ != nullptr; }
  std::uint32_t used() const noexcept {
    return hdr_->used.load(std::memory_order_acquire);
  }

  // Publisher side: create the topic if it does not exist, or attach to it if
  // it does and the layout agrees.
  //
  // `exclusive` claims the topic's single publisher slot. The ring protocol is
  // only sound with one publisher per topic -- TLC finds a torn read in the
  // multi-publisher case, see the header comment in shm/ring.hpp -- so the
  // second publisher is refused rather than allowed to corrupt the ring. Pass
  // false only to look the topic up without claiming it.
  template <class T>
  attach_status announce(std::string_view name, topic_entry** out,
                         bool exclusive = false, std::uint32_t pid = 0) noexcept {
    static_assert(message_traits<T>::declared,
                  "announce<T>() needs a LOCKSTEP_MESSAGE type");
    if (name.size() > max_topic_name) return attach_status::name_too_long;

    // Fast path: the topic already exists, so no lock is needed.
    if (topic_entry* existing = find(name)) {
      *out = existing;
      const attach_status st = verify<T>(*existing);
      if (st != attach_status::ok) return st;
      return exclusive ? claim_publisher(*existing, pid) : attach_status::ok;
    }

    // Creation is serialised; see registry_header::create_lock.
    detail::create_guard guard(*hdr_);

    // Re-check under the lock: somebody may have created it while we waited.
    if (topic_entry* existing = find(name)) {
      *out = existing;
      const attach_status st = verify<T>(*existing);
      if (st != attach_status::ok) return st;
      return exclusive ? claim_publisher(*existing, pid) : attach_status::ok;
    }

    for (std::uint32_t i = 0; i < hdr_->capacity; ++i) {
      topic_entry& e = hdr_->topics[i];
      std::uint32_t expected = static_cast<std::uint32_t>(topic_state::empty);
      if (!e.state.compare_exchange_strong(
              expected, static_cast<std::uint32_t>(topic_state::claiming),
              std::memory_order_acq_rel, std::memory_order_acquire)) {
        continue;  // somebody else owns this slot
      }

      e.name_len = static_cast<std::uint32_t>(name.size());
      std::memcpy(e.name, name.data(), name.size());
      if (name.size() < max_topic_name)
        std::memset(e.name + name.size(), 0, max_topic_name - name.size());

      e.layout_hash = message_traits<T>::layout_hash;
      e.message_size = message_traits<T>::size;
      e.message_align = static_cast<std::uint32_t>(message_traits<T>::align);
      e.message_has_padding = message_traits<T>::has_padding ? 1u : 0u;
      e.ring_offset.store(0, std::memory_order_relaxed);
      e.publisher_pid.store(0, std::memory_order_relaxed);
      e.heartbeat_ns.store(0, std::memory_order_relaxed);

      hdr_->used.fetch_add(1, std::memory_order_acq_rel);
      // Release: everything written above must be visible to whoever sees ready.
      e.state.store(static_cast<std::uint32_t>(topic_state::ready),
                    std::memory_order_release);
      *out = &e;
      return exclusive ? claim_publisher(e, pid) : attach_status::ok;
    }
    return attach_status::registry_full;
  }

  // Subscriber side: the topic must already exist and must agree on layout.
  template <class T>
  attach_status attach(std::string_view name, topic_entry** out) noexcept {
    static_assert(message_traits<T>::declared,
                  "attach<T>() needs a LOCKSTEP_MESSAGE type");
    topic_entry* e = find(name);
    if (e == nullptr) return attach_status::not_found;
    *out = e;
    return verify<T>(*e);
  }

  topic_entry* find(std::string_view name) noexcept {
    for (std::uint32_t i = 0; i < hdr_->capacity; ++i) {
      topic_entry& e = hdr_->topics[i];
      if (e.state.load(std::memory_order_acquire) !=
          static_cast<std::uint32_t>(topic_state::ready)) {
        continue;
      }
      if (e.name_len == name.size() &&
          std::memcmp(e.name, name.data(), name.size()) == 0) {
        return &e;
      }
    }
    return nullptr;
  }

  topic_entry& entry(std::uint32_t i) noexcept { return hdr_->topics[i]; }
  std::uint32_t capacity() const noexcept { return hdr_->capacity; }

 private:
  // Exactly one process may hold a topic's publisher slot at a time. Claiming
  // it again from the same pid is idempotent, so a publisher may re-announce.
  static attach_status claim_publisher(topic_entry& e, std::uint32_t pid) noexcept {
    std::uint32_t expected = 0;
    if (e.publisher_pid.compare_exchange_strong(expected, pid,
                                                std::memory_order_acq_rel,
                                                std::memory_order_acquire)) {
      return attach_status::ok;
    }
    return (expected == pid) ? attach_status::ok : attach_status::already_published;
  }

  template <class T>
  static attach_status verify(const topic_entry& e) noexcept {
    if (e.layout_hash != message_traits<T>::layout_hash)
      return attach_status::layout_mismatch;
    if (e.message_size != message_traits<T>::size)
      return attach_status::size_mismatch;
    return attach_status::ok;
  }

  registry_header* hdr_ = nullptr;
};

}  // namespace ls
