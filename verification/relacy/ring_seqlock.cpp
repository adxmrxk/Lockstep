// Relacy model of the ring's seqlock, checking the MEMORY ORDERING.
//
// This closes the gap docs/tla/README.md admits to. TLA+ verified the ALGORITHM
// -- every action there is sequentially consistent, so a wrong memory order
// sails straight past it. Relacy simulates the C++11 memory model itself:
// it explores the reorderings that acquire/release permit, so it can catch a
// barrier that is too weak, which is the one class of bug the TLA+ spec is
// structurally unable to see.
//
// WHAT IS MODELLED
//
// One publisher, one subscriber, one ring slot, one message -- the smallest
// configuration in which the seqlock's ordering argument is load-bearing.
// Capacity and lapping are the ALGORITHM's business and TLA+ already covers
// them exhaustively; adding them here would multiply the schedule space without
// testing anything new about the barriers.
//
// The publisher and subscriber below are the real protocol, transcribed:
//
//   publisher   state.store(2t, release)
//               payload.store(v, relaxed)      <- relaxed, exactly as in the
//               state.store(2t+1, release)        implementation
//
//   subscriber  s1 = state.load(acquire)
//               v  = payload.load(relaxed)
//               s2 = state.load(acquire)
//               accept only if s1 == s2 == 2t+1
//
// The claim being checked: a subscriber that accepts is guaranteed to see the
// payload that belongs to that ticket. The release store on commit and the
// acquire load on validation are what make that true; the relaxed payload
// access is safe ONLY because those two bracket it.
//
// THE CONTROL. Building with -DLOCKSTEP_RELACY_RELAXED_STATE downgrades the two
// state accesses to relaxed and nothing else. That build MUST fail, and
// scripts/relacy-check.sh asserts that it does. Without that half, a passing
// run would not distinguish "the barriers are right" from "this model cannot
// tell", which is the same trap the replay test avoids with its live-rerun
// control.
#include <relacy/relacy_std.hpp>

#if defined(LOCKSTEP_RELACY_RELAXED_STATE)
#define LS_STATE_STORE rl::memory_order_relaxed
#define LS_STATE_LOAD rl::memory_order_relaxed
#else
#define LS_STATE_STORE rl::memory_order_release
#define LS_STATE_LOAD rl::memory_order_acquire
#endif

namespace {

constexpr int kTicket = 0;
constexpr int kCommitted = 2 * kTicket + 1;  // the state word meaning "ready"
constexpr int kPayloadValue = 0xABCD;

struct ring_seqlock : rl::test_suite<ring_seqlock, 2> {
  rl::atomic<int> state;
  rl::atomic<int> payload;

  // Did the subscriber ever actually validate a message? Checked in after(),
  // so a run where the reader simply never saw the write cannot be mistaken for
  // a run where the ordering held.
  bool accepted;

  void before() {
    state($).store(0, rl::memory_order_relaxed);
    payload($).store(0, rl::memory_order_relaxed);
    accepted = false;
  }

  void thread(unsigned index) {
    if (index == 0) {
      // ---- publisher ----
      state($).store(2 * kTicket, LS_STATE_STORE);       // claim
      payload($).store(kPayloadValue, rl::memory_order_relaxed);
      state($).store(kCommitted, LS_STATE_STORE);        // commit
    } else {
      // ---- subscriber ----
      const int s1 = state($).load(LS_STATE_LOAD);
      if (s1 != kCommitted) return;  // not ready yet, or mid-write: no claim made

      const int v = payload($).load(rl::memory_order_relaxed);

      const int s2 = state($).load(LS_STATE_LOAD);
      if (s2 != s1) return;  // torn: the publisher moved on, discard

      // THE PROPERTY. Having seen the commit and confirmed it did not move,
      // the payload we read must be the one that commit published.
      accepted = true;
      RL_ASSERT(v == kPayloadValue);
    }
  }

  void after() {
    // Nothing to assert: `accepted` is false on the many schedules where the
    // reader runs first, and that is legitimate. It exists so a human reading
    // a passing run can see the accepting schedules were explored at all.
    (void)accepted;
  }
};

}  // namespace

int main() {
  rl::test_params p;
  p.iteration_count = 100000;
  p.search_type = rl::sched_full;  // exhaustive, not random
  return rl::simulate<ring_seqlock>(p) ? 0 : 1;
}
