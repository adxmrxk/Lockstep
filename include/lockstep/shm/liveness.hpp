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
// Liveness itself lives in core/process.hpp, because the registry needs it too
// and cannot include this header. Its caveats -- zombies, pid recycling, and
// what Windows does instead -- are documented there.
#pragma once

#include <cstdint>

#include "lockstep/core/process.hpp"
#include "lockstep/shm/bus.hpp"
#include "lockstep/shm/registry.hpp"
#include "lockstep/shm/ring.hpp"

namespace ls {

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

  // Repair every ticket the dead publisher took but never delivered.
  //
  // There are TWO ways a ticket can be left undelivered, and only handling the
  // obvious one leaves a hole that wedges subscribers permanently:
  //
  //   state == 2t  the publisher claimed the slot and died before committing.
  //                This is the obvious case.
  //
  //   state <  2t  the publisher took the ticket from reserve() and died before
  //                it even stored the claim, so the slot still holds the state
  //                of an OLDER ticket, capacity publications ago. This one is
  //                easy to miss: the window is two instructions wide, so on
  //                Linux it almost never happens. It showed up as soon as the
  //                same test ran on Windows, where process teardown is slower
  //                and it reproduced in 2 runs out of 6.
  //
  // The second case is the damaging one. A subscriber at cursor t loads a state
  // word BELOW 2t+1, reads that as "nothing published yet", and waits -- forever,
  // because the only process that could ever have published t is dead. One such
  // ticket stalls every subscriber on the topic at that point in the stream.
  //
  //   state >  2t  is not damage: the slot has been overwritten by a later
  //                ticket, which is an ordinary lap, and read() already reports
  //                it as an overrun. Left alone.
  //
  // Neither case may simply be committed as-is: the bytes are either
  // half-written or belong to another message entirely. They are marked
  // abandoned instead -- committed so readers stop waiting, flagged so readers
  // skip them rather than trusting the contents.
  static std::uint32_t heal(ring& rg) noexcept {
    std::uint32_t healed = 0;
    const std::uint64_t wp = rg.write_pos();
    const std::uint64_t cap = rg.capacity();
    const std::uint64_t first = (wp > cap) ? wp - cap : 0;

    for (std::uint64_t t = first; t < wp; ++t) {
      ring_slot& s = rg.slot(t);
      std::uint64_t st = s.state.load(std::memory_order_acquire);
      if (st > 2 * t) continue;  // committed for t, or lapped by a later ticket

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
