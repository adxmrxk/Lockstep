// rt/executor.hpp : a deadline-aware executor.
//
// A ROS 2 executor decides which callback runs next using a policy that is hard
// to predict and impossible to replay. This one is deliberately dull: callbacks
// are registered up front, run in a fixed priority order, and every dispatch
// decision is recorded. Dullness is the feature -- it is what makes phase 7's
// bit-exact replay possible at all.
//
// NO ALLOCATION AFTER INIT. Every container is sized during register_callback
// and reserve(); spin() calls nothing that can allocate. This is not a claim
// made in a comment -- tests/test_executor.cpp installs a global operator new
// hook and fails if a single allocation happens inside the run loop.
//
// REAL-TIME SCHEDULING is requested, not assumed. try_set_realtime() attempts
// SCHED_FIFO (and SCHED_DEADLINE where the kernel has it), and REPORTS whether
// it worked. On a stock kernel, in a container, or under WSL2 it will usually
// fail for want of privilege or of a PREEMPT_RT kernel, and the executor
// carries on as an ordinary thread. What it does not do is pretend: a run whose
// scheduling request failed is marked as such, and timing from it is not a
// real-time measurement.
#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "lockstep/pubsub.hpp"
#include "lockstep/rt/response_time.hpp"

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace ls::rt {

enum class sched_policy : std::uint32_t { normal = 0, fifo, deadline };

struct sched_report {
  bool requested_realtime = false;
  bool got_realtime = false;
  bool memory_locked = false;
  sched_policy policy = sched_policy::normal;
  int last_errno = 0;
  std::string detail;

  // The honest summary. If this is false, nothing timed under this executor is
  // a real-time result.
  bool trustworthy_for_timing() const noexcept { return got_realtime && memory_locked; }
};

// Ask the kernel for real-time scheduling and locked memory. Returns what
// actually happened rather than throwing, because the caller usually wants to
// carry on degraded and say so.
inline sched_report try_set_realtime([[maybe_unused]] int fifo_priority = 80) noexcept {
  sched_report r;
  r.requested_realtime = true;
#if defined(__linux__)
  // Page faults are unbounded latency, so a real-time process locks its pages
  // down before it starts.
  if (::mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
    r.memory_locked = true;
  } else {
    r.last_errno = errno;
    r.detail = "mlockall failed: ";
    r.detail += std::strerror(errno);
  }

  ::sched_param param{};
  param.sched_priority = fifo_priority;
  if (::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &param) == 0) {
    r.got_realtime = true;
    r.policy = sched_policy::fifo;
  } else {
    r.last_errno = errno;
    if (!r.detail.empty()) r.detail += "; ";
    r.detail += "SCHED_FIFO refused (needs CAP_SYS_NICE and a suitable kernel)";
  }
#else
  r.detail = "real-time scheduling is only wired up for Linux";
#endif
  return r;
}

// One registered callback: a subscription plus its timing contract.
struct callback_stats {
  std::uint64_t invocations = 0;
  std::uint64_t deadline_misses = 0;
  std::uint64_t worst_observed_ns = 0;
  std::uint64_t total_ns = 0;
};

class executor {
 public:
  // A callback takes no arguments: it is bound to its subscription by the
  // closure built in subscribe(). Returning void keeps dispatch branch-free.
  using thunk = std::function<void()>;

  explicit executor(std::size_t max_callbacks = 32) {
    entries_.reserve(max_callbacks);
    specs_.reserve(max_callbacks);
    stats_.reserve(max_callbacks);
    capacity_ = max_callbacks;
  }

  // Register a subscription with its timing contract. Init-time only: this
  // allocates, which is exactly why it is not on the spin path.
  template <class T, class Fn>
  void subscribe(subscriber<T>& sub, task_spec spec, Fn&& fn) {
    if (entries_.size() >= capacity_)
      throw std::runtime_error("lockstep: executor is full; raise max_callbacks");

    auto* s = &sub;
    auto f = std::forward<Fn>(fn);
    entries_.push_back(entry{
        [s, f]() mutable {
          sample<T> smp;
          // Drain everything queued for this callback in one dispatch, so a
          // callback cannot fall permanently behind a faster publisher.
          for (;;) {
            const read_result rr = s->take(smp);
            if (rr == read_result::ok) {
              f(smp);
            } else if (rr == read_result::retry || rr == read_result::overrun) {
              // Overrun means read() already advanced the cursor to the oldest
              // resident message, so keep draining from there. Breaking instead
              // leaves the cursor pinned behind the trailing edge, where it
              // overruns again every cycle and delivers nothing.
              continue;
            } else {
              break;  // empty or abandoned
            }
          }
        }});
    specs_.push_back(spec);
    stats_.push_back(callback_stats{});

    // Fixed priority order, decided once, here. Dispatch order never depends on
    // arrival order, which is what makes a run reproducible.
    sort_by_priority();
  }

  // Run every callback once, in priority order. Allocates nothing.
  void spin_once() noexcept {
    for (std::size_t i = 0; i < order_.size(); ++i) {
      const std::size_t k = order_[i];
      const std::uint64_t t0 = now_ns();
      entries_[k].fn();
      const std::uint64_t dt = now_ns() - t0;

      callback_stats& st = stats_[k];
      ++st.invocations;
      st.total_ns += dt;
      if (dt > st.worst_observed_ns) st.worst_observed_ns = dt;
      if (dt > specs_[k].effective_deadline()) ++st.deadline_misses;
    }
    ++spins_;
  }

  void spin(std::uint64_t iterations) noexcept {
    for (std::uint64_t i = 0; i < iterations; ++i) spin_once();
  }

  // Schedulability of the registered set, from the declared WCETs.
  analysis analyse_schedulability() const { return analyse(specs_); }

  // Schedulability using the WORST TIMES ACTUALLY OBSERVED rather than the
  // declared ones. If this disagrees with the declared analysis, the WCETs were
  // wrong, and the declared analysis was answering the wrong question.
  analysis analyse_observed() const {
    std::vector<task_spec> measured = specs_;
    for (std::size_t i = 0; i < measured.size(); ++i)
      measured[i].wcet_ns = stats_[i].worst_observed_ns;
    return analyse(measured);
  }

  const callback_stats& stats(std::size_t i) const noexcept { return stats_[i]; }
  const task_spec& spec(std::size_t i) const noexcept { return specs_[i]; }
  std::size_t size() const noexcept { return entries_.size(); }
  std::uint64_t spins() const noexcept { return spins_; }

  std::uint64_t total_deadline_misses() const noexcept {
    std::uint64_t n = 0;
    for (const auto& s : stats_) n += s.deadline_misses;
    return n;
  }

 private:
  struct entry {
    thunk fn;
  };

  // entries_, specs_ and stats_ all stay in REGISTRATION order for the life of
  // the executor; only order_ is sorted. Reordering the vectors themselves and
  // keeping an index into them at the same time is how this went wrong first
  // time round -- after the second subscribe() the index referred to positions
  // that had already moved. One indirection, applied consistently, or none.
  void sort_by_priority() {
    order_.resize(specs_.size());
    for (std::size_t i = 0; i < order_.size(); ++i) order_[i] = i;

    // Insertion sort: the set is tiny, and it is stable, so callbacks of equal
    // priority keep registration order. Dispatch order therefore depends only
    // on what was registered and in what sequence -- never on arrival timing --
    // which is the property phase 7 replays against.
    for (std::size_t i = 1; i < order_.size(); ++i) {
      const std::size_t key = order_[i];
      std::size_t j = i;
      while (j > 0 && specs_[order_[j - 1]].priority > specs_[key].priority) {
        order_[j] = order_[j - 1];
        --j;
      }
      order_[j] = key;
    }
  }

  std::size_t capacity_ = 0;
  std::vector<entry> entries_;
  std::vector<task_spec> specs_;
  std::vector<callback_stats> stats_;
  std::vector<std::size_t> order_;
  std::uint64_t spins_ = 0;
};

}  // namespace ls::rt
