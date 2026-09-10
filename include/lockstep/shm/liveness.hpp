// shm/liveness.hpp : detecting dead publishers and repairing what they left.
//
// A publisher killed with SIGKILL gets no chance to clean up. It can die:
//
//   * holding a topic's exclusive publisher slot, so no replacement can ever
//     take over that topic;
//   * between claiming a ring slot (state := 2t) and committing it
//     (state := 2t+1), leaving that slot mid-write forever.
//
// Neither corrupts anything -- the seqlock means a subscriber never accepts a
// half-written slot, it just never accepts THAT ticket -- but both wedge the
// topic, which for a robot is the same thing as broken. So the bus needs
// something that notices and repairs.
//
// LIVENESS. process_alive() is kill(pid, 0): it asks the kernel whether a pid
// exists and we may signal it. Two known limits, both stated rather than
// papered over:
//
//   * A zombie -- dead but unreaped by its parent -- still answers yes. On this
//     bus that is the right answer anyway: a zombie has certainly stopped
//     publishing, but its pid is not yet reusable, so nothing else can be
//     confused for it.
//   * A pid can in principle be recycled onto a different process. The window
//     is enormous compared to a heartbeat interval, and the heartbeat check
//     below closes it in practice: a recycled pid will not be updating this
//     topic's heartbeat.
//
// pidfd_open (Linux 5.3+) closes the recycling hole properly by pinning the
// process rather than its number. It is used when available.
#pragma once

#include <cerrno>
#include <cstdint>
#include <csignal>

#include "lockstep/shm/bus.hpp"
#include "lockstep/shm/registry.hpp"
#include "lockstep/shm/ring.hpp"

#if defined(__linux__)
#include <sys/syscall.h>
#include <unistd.h>
#endif

namespace ls {

inline bool process_alive(std::uint32_t pid) noexcept {
  if (pid == 0) return false;
#if defined(__linux__) && defined(SYS_pidfd_open)
  // pidfd_open refers to the process, not the number, so it cannot be fooled by
  // pid recycling. ESRCH means gone; EPERM means alive but not ours.
  const long fd = ::syscall(SYS_pidfd_open, static_cast<int>(pid), 0u);
  if (fd >= 0) {
    ::close(static_cast<int>(fd));
    return true;
  }
  if (errno == ESRCH) return false;
  if (errno == EPERM) return true;
  // ENOSYS on an older kernel: fall through to kill(2).
#endif
  return ::kill(static_cast<int>(pid), 0) == 0 || errno == EPERM;
}

// Why a reap pass did or did not act on a topic.
struct reap_report {
  std::uint32_t topics_scanned = 0;
  std::uint32_t publishers_reclaimed = 0;  // dead publisher's slot freed
  std::uint32_t slots_healed = 0;          // ring slots left mid-write
};

// Repairs a bus after a publisher dies. Safe to run from any process at any
// time, including concurrently with live publishers on other topics: it only
// ever touches a topic whose publisher it has established is gone.
class reaper {
 public:
  reaper() = default;
  explicit reaper(bus& b) noexcept : bus_(&b) {}

  reap_report scan() noexcept {
    reap_report r;
    registry& reg = bus_->topics();

    for (std::uint32_t i = 0; i < reg.capacity(); ++i) {
      topic_entry& e = reg.entry(i);
      if (e.state.load(std::memory_order_acquire) !=
          static_cast<std::uint32_t>(topic_state::ready)) {
        continue;
      }
      ++r.topics_scanned;

      const std::uint32_t pid = e.publisher_pid.load(std::memory_order_acquire);
      if (pid == 0 || process_alive(pid)) continue;

      // The publisher is gone. Heal its ring before releasing the topic, so a
      // replacement publisher never inherits a slot stuck mid-write.
      const std::uint64_t ro = e.ring_offset.load(std::memory_order_acquire);
      if (ro != 0) {
        ring rg(bus_->seg().template at<ring_header>(ro));
        r.slots_healed += heal(rg);
      }

      // Release the publisher slot, but only if it is still the pid we
      // diagnosed: another reaper may have got there first, and a replacement
      // publisher may already have claimed it.
      std::uint32_t expected = pid;
      if (e.publisher_pid.compare_exchange_strong(expected, 0,
                                                  std::memory_order_acq_rel,
                                                  std::memory_order_acquire)) {
        ++r.publishers_reclaimed;
      }
    }
    return r;
  }

  // A slot whose state is EVEN was claimed and never committed, which can only
  // mean the publisher died between the two stores. Advancing it to the odd
  // value would publish whatever half-written bytes are in there, so instead it
  // is marked abandoned: committed, so readers stop waiting on it, and flagged,
  // so readers skip it rather than trusting it.
  static std::uint32_t heal(ring& rg) noexcept {
    std::uint32_t healed = 0;
    const std::uint64_t wp = rg.write_pos();
    const std::uint64_t cap = rg.capacity();
    const std::uint64_t first = (wp > cap) ? wp - cap : 0;

    for (std::uint64_t t = first; t < wp; ++t) {
      ring_slot& s = rg.slot(t);
      std::uint64_t st = s.state.load(std::memory_order_acquire);
      if (st != 2 * t) continue;  // committed, or reused by a later ticket

      s.flags.store(slot_abandoned, std::memory_order_relaxed);
      if (s.state.compare_exchange_strong(st, 2 * t + 1,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire)) {
        ++healed;
      }
    }
    return healed;
  }

 private:
  bus* bus_ = nullptr;
};

}  // namespace ls
