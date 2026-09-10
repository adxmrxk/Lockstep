// The deadline-aware executor and its response-time analysis.
//
// Two things here are worth more than the rest:
//
//   * the no-allocation claim is enforced by a global operator new hook, so
//     "allocation-free on the hot path" is a test result rather than a comment;
//   * the response-time recurrence is checked against a worked example with
//     known answers, so a subtly wrong ceil or comparison shows up as a wrong
//     number rather than as a plausible one.
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <string>
#include <unistd.h>
#include <vector>

#include "lockstep/rt/executor.hpp"
#include "support/check.hpp"
#include "support/demo_msgs.hpp"

// ---------------------------------------------------------------------------
// Allocation tripwire. Armed only around the region under test, so the harness
// and the framework's own allocations do not pollute the count.
// ---------------------------------------------------------------------------
namespace alloc_watch {
std::atomic<bool> armed{false};
std::atomic<long> count{0};
}  // namespace alloc_watch

void* operator new(std::size_t n) {
  if (alloc_watch::armed.load(std::memory_order_relaxed))
    alloc_watch::count.fetch_add(1, std::memory_order_relaxed);
  void* p = std::malloc(n ? n : 1);
  if (p == nullptr) throw std::bad_alloc();
  return p;
}
void* operator new[](std::size_t n) { return ::operator new(n); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

namespace {

std::string unique_name(const char* tag) {
  return std::string("lockstep-exec-") + tag + "-" + std::to_string(::getpid());
}

ls::bus make_bus(const char* tag) {
  const std::string name = unique_name(tag);
  ls::segment::unlink(name);
  return ls::bus::create(name, {{64, 256}, {4096, 16}, {65536, 4}});
}

// ---------------------------------------------------------------------------
// Response-time analysis, against a worked example.
//
//   T1: period 7,  C 3, highest priority
//   T2: period 12, C 3
//   T3: period 20, C 5, lowest
//
// By hand:
//   R1 = 3
//   R2 = 3 + ceil(3/7)*3 = 6, then stable at 6
//   R3 = 5 -> 11 -> 14 -> 17 -> 20, stable at 20, exactly meeting its deadline
//   U  = 3/7 + 3/12 + 5/20 = 0.9286
// ---------------------------------------------------------------------------
void response_time_matches_the_worked_example() {
  const std::vector<ls::rt::task_spec> tasks = {
      {7000, 0, 3000, 0, "t1"},
      {12000, 0, 3000, 1, "t2"},
      {20000, 0, 5000, 2, "t3"},
  };

  const auto a = ls::rt::analyse(tasks);
  LS_CHECK(a.schedulable);
  LS_CHECK_EQ(a.tasks.size(), 3u);
  LS_CHECK_EQ(a.tasks[0].response_ns, 3000u);
  LS_CHECK_EQ(a.tasks[1].response_ns, 6000u);
  LS_CHECK_EQ(a.tasks[2].response_ns, 20000u);
  for (const auto& t : a.tasks) LS_CHECK(t.converged);

  LS_CHECK(a.utilisation > 0.92 && a.utilisation < 0.93);
}

// One unit more of work in the lowest-priority task and the set stops being
// schedulable, even though utilisation is still under 1.
void one_more_unit_of_work_breaks_it() {
  const std::vector<ls::rt::task_spec> tasks = {
      {7000, 0, 3000, 0, "t1"},
      {12000, 0, 3000, 1, "t2"},
      {20000, 0, 6000, 2, "t3"},
  };

  const auto a = ls::rt::analyse(tasks);
  LS_CHECK(!a.schedulable);
  LS_CHECK(a.tasks[0].schedulable);
  LS_CHECK(a.tasks[1].schedulable);
  LS_CHECK(!a.tasks[2].schedulable);

  // Utilisation alone would not have caught this: it is still below 1.
  LS_CHECK(a.utilisation < 1.0);
}

void an_overloaded_set_is_rejected() {
  const std::vector<ls::rt::task_spec> tasks = {
      {1000, 0, 800, 0, "a"},
      {1000, 0, 800, 1, "b"},
  };
  const auto a = ls::rt::analyse(tasks);
  LS_CHECK(!a.schedulable);
  LS_CHECK(a.utilisation > 1.0);
}

// A tighter deadline than the period must be honoured.
void a_constrained_deadline_is_respected() {
  const std::vector<ls::rt::task_spec> loose = {{100, 100, 60, 0, "x"}};
  LS_CHECK(ls::rt::analyse(loose).schedulable);

  const std::vector<ls::rt::task_spec> tight = {{100, 50, 60, 0, "x"}};
  LS_CHECK(!ls::rt::analyse(tight).schedulable);
}

// ---------------------------------------------------------------------------
// The executor
// ---------------------------------------------------------------------------

void callbacks_run_in_priority_order_regardless_of_registration_order() {
  ls::bus b = make_bus("order");
  auto pub = ls::publisher<ImuSample>::create(b, "imu", 16);
  auto sub_lo = ls::subscriber<ImuSample>::attach(b, "imu");
  auto sub_hi = ls::subscriber<ImuSample>::attach(b, "imu");

  std::vector<int> fired;
  ls::rt::executor ex;

  // Registered low-priority first, on purpose.
  ex.subscribe(sub_lo, ls::rt::task_spec{1000000, 0, 1000, 9, "low"},
               [&](const ls::sample<ImuSample>&) { fired.push_back(9); });
  ex.subscribe(sub_hi, ls::rt::task_spec{1000000, 0, 1000, 1, "high"},
               [&](const ls::sample<ImuSample>&) { fired.push_back(1); });

  auto ln = pub.loan_message();
  ln->stamp_ns = 1;
  pub.publish(ln);

  ex.spin_once();

  LS_CHECK_EQ(fired.size(), 2u);
  if (fired.size() == 2) {
    LS_CHECK_EQ(fired[0], 1);  // high priority ran first
    LS_CHECK_EQ(fired[1], 9);
  }
}

void the_executor_delivers_messages_to_callbacks() {
  ls::bus b = make_bus("deliver");
  auto pub = ls::publisher<ImuSample>::create(b, "imu", 16);
  auto sub = ls::subscriber<ImuSample>::attach(b, "imu");

  std::uint64_t seen = 0;
  std::uint64_t sum = 0;
  ls::rt::executor ex;
  ex.subscribe(sub, ls::rt::task_spec{1000000, 0, 100000, 0, "c"},
               [&](const ls::sample<ImuSample>& s) {
                 ++seen;
                 sum += s->stamp_ns;
               });

  for (std::uint64_t i = 1; i <= 5; ++i) {
    auto ln = pub.loan_message();
    ln->stamp_ns = i;
    pub.publish(ln);
  }

  ex.spin_once();  // one dispatch drains everything queued
  LS_CHECK_EQ(seen, 5u);
  LS_CHECK_EQ(sum, 15u);
  LS_CHECK_EQ(ex.stats(0).invocations, 1u);
}

// THE claim: nothing on the spin path allocates.
void spinning_allocates_nothing() {
  ls::bus b = make_bus("noalloc");
  auto pub = ls::publisher<ImuSample>::create(b, "imu", 64);
  auto sub = ls::subscriber<ImuSample>::attach(b, "imu");

  std::uint64_t seen = 0;
  ls::rt::executor ex;
  ex.subscribe(sub, ls::rt::task_spec{1000000, 0, 1000000, 0, "c"},
               [&](const ls::sample<ImuSample>&) { ++seen; });

  // Publish first, so the spin below has real work rather than hitting an
  // empty ring and returning immediately.
  for (int i = 0; i < 32; ++i) {
    auto ln = pub.loan_message();
    ln->stamp_ns = static_cast<std::uint64_t>(i);
    pub.publish(ln);
  }

  alloc_watch::count.store(0, std::memory_order_relaxed);
  alloc_watch::armed.store(true, std::memory_order_relaxed);
  ex.spin(100);
  alloc_watch::armed.store(false, std::memory_order_relaxed);

  std::fprintf(stderr, "  [executor] %llu callbacks over 100 spins, %ld allocations\n",
               (unsigned long long)seen, alloc_watch::count.load());

  LS_CHECK(seen > 0);  // it really did work while the tripwire was armed
  LS_CHECK_EQ(alloc_watch::count.load(), 0L);
}

// Publishing must be allocation-free too, or the publisher is the thing that
// blows the deadline.
void publishing_allocates_nothing() {
  ls::bus b = make_bus("noallocpub");
  auto pub = ls::publisher<ImuSample>::create(b, "imu", 64);

  alloc_watch::count.store(0, std::memory_order_relaxed);
  alloc_watch::armed.store(true, std::memory_order_relaxed);
  for (int i = 0; i < 500; ++i) {
    auto ln = pub.loan_message();
    ln->stamp_ns = static_cast<std::uint64_t>(i);
    pub.publish(ln);
  }
  alloc_watch::armed.store(false, std::memory_order_relaxed);

  LS_CHECK_EQ(alloc_watch::count.load(), 0L);
  LS_CHECK_EQ(pub.published(), 500u);
}

void a_callback_that_overruns_its_deadline_is_counted() {
  ls::bus b = make_bus("miss");
  auto pub = ls::publisher<ImuSample>::create(b, "imu", 16);
  auto sub = ls::subscriber<ImuSample>::attach(b, "imu");

  ls::rt::executor ex;
  // A 1ns deadline that any real callback must miss.
  ex.subscribe(sub, ls::rt::task_spec{1000000, 1, 1, 0, "impossible"},
               [&](const ls::sample<ImuSample>&) {
                 volatile std::uint64_t x = 0;
                 for (int i = 0; i < 20000; ++i) x += static_cast<std::uint64_t>(i);
                 (void)x;
               });

  auto ln = pub.loan_message();
  ln->stamp_ns = 1;
  pub.publish(ln);
  ex.spin_once();

  LS_CHECK(ex.total_deadline_misses() > 0);
  LS_CHECK(ex.stats(0).worst_observed_ns > 0);
}

// The scheduling request must report what really happened. On WSL2 it is
// expected to fail, and the point of the test is that it SAYS so.
void realtime_scheduling_reports_honestly() {
  const ls::rt::sched_report r = ls::rt::try_set_realtime();
  std::fprintf(stderr,
               "  [executor] realtime requested=%d got=%d mlock=%d trustworthy=%d\n"
               "  [executor] detail: %s\n",
               (int)r.requested_realtime, (int)r.got_realtime, (int)r.memory_locked,
               (int)r.trustworthy_for_timing(),
               r.detail.empty() ? "(none)" : r.detail.c_str());

  LS_CHECK(r.requested_realtime);
  // Whatever the outcome, trustworthy_for_timing must agree with it rather than
  // being optimistic.
  LS_CHECK_EQ(r.trustworthy_for_timing(), r.got_realtime && r.memory_locked);
}

}  // namespace

int main() {
  response_time_matches_the_worked_example();
  one_more_unit_of_work_breaks_it();
  an_overloaded_set_is_rejected();
  a_constrained_deadline_is_respected();
  callbacks_run_in_priority_order_regardless_of_registration_order();
  the_executor_delivers_messages_to_callbacks();
  spinning_allocates_nothing();
  publishing_allocates_nothing();
  a_callback_that_overruns_its_deadline_is_counted();
  realtime_scheduling_reports_honestly();
  return ls::test::summary("executor");
}
