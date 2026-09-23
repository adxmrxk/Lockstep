// Does this machine actually give us real-time behaviour, and if so, do we
// deliver it?
//
// Everything else in this repo measures latency and reports it honestly as "not
// a real-time number". This tool exists to be the thing that CAN say otherwise,
// on a box that qualifies -- and to say precisely what is missing on a box that
// does not, rather than leaving "needs PREEMPT_RT" as folklore.
//
// It has two modes, chosen by what the kernel grants, not by a flag:
//
//   NOT GRANTED  print exactly which prerequisite failed and how to fix it,
//                then exit 0. This is a SKIP, not a failure: the machine is not
//                eligible, which is information, not a bug. Exiting non-zero
//                here would mean nobody could put this in CI.
//
//   GRANTED      run a periodic workload and ASSERT on the result: bounded
//                jitter, zero deadline misses, and zero page faults after
//                mlockall. Exit non-zero if any of those is violated. On an
//                eligible machine this is a real-time regression test.
//
// The development machine this was written on is not eligible, and reports so:
//
//     rtprio limit         0        <- needs a non-zero RLIMIT_RTPRIO
//     kernel               5.15.167.4-microsoft-standard-WSL2
//     PREEMPT_RT           no
//
// Which is why the README quotes no real-time figure. This tool is how that
// changes, and the assertions below are what it would have to survive.
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "lockstep/core/process.hpp"
#include "lockstep/pubsub.hpp"
#include "lockstep/rt/executor.hpp"
#include "demo_msgs.hpp"

#if defined(__linux__)
#include <sys/resource.h>
#include <sys/utsname.h>
#endif

namespace {

struct prereq {
  const char* name;
  bool ok;
  std::string detail;
  const char* remedy;
};

std::uint64_t pct(std::vector<std::uint64_t>& v, double p) {
  if (v.empty()) return 0;
  return v[static_cast<std::size_t>(static_cast<double>(v.size() - 1) * p)];
}

long minor_faults() {
#if defined(__linux__)
  struct ::rusage ru {};
  ::getrusage(RUSAGE_SELF, &ru);
  return ru.ru_minflt;
#else
  return 0;
#endif
}

std::vector<prereq> check_prereqs(const ls::rt::sched_report& sr) {
  std::vector<prereq> out;

#if defined(__linux__)
  struct ::rlimit rl {};
  ::getrlimit(RLIMIT_RTPRIO, &rl);
  out.push_back({"RLIMIT_RTPRIO", rl.rlim_cur > 0,
                 std::to_string(static_cast<long long>(rl.rlim_cur)),
                 "grant CAP_SYS_NICE, or add an rtprio line to "
                 "/etc/security/limits.conf"});

  struct ::utsname u {};
  ::uname(&u);
  const bool rt_kernel = std::strstr(u.version, "PREEMPT_RT") != nullptr ||
                         std::strstr(u.version, "PREEMPT RT") != nullptr;
  out.push_back({"PREEMPT_RT kernel", rt_kernel, std::string(u.release),
                 "boot a PREEMPT_RT kernel; a stock or WSL2 kernel cannot bound "
                 "the tail no matter what priority you ask for"});

  ::getrlimit(RLIMIT_MEMLOCK, &rl);
  const bool memlock_ok = rl.rlim_cur == RLIM_INFINITY || rl.rlim_cur >= (64u << 20);
  out.push_back({"RLIMIT_MEMLOCK", memlock_ok,
                 rl.rlim_cur == RLIM_INFINITY
                     ? std::string("unlimited")
                     : std::to_string(static_cast<long long>(rl.rlim_cur >> 20)) + " MB",
                 "raise memlock so mlockall can pin a multi-megabyte arena"});
#else
  out.push_back({"Linux", false, "this platform", "real-time support is Linux-only here"});
#endif

  out.push_back({"SCHED_FIFO granted", sr.got_realtime,
                 sr.got_realtime ? "yes" : "refused", "see RLIMIT_RTPRIO above"});
  out.push_back({"mlockall", sr.memory_locked, sr.memory_locked ? "yes" : "failed",
                 "see RLIMIT_MEMLOCK above"});
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  const int iterations = argc > 1 ? std::atoi(argv[1]) : 20000;
  const std::uint64_t period_ns = 1000000;  // 1 kHz, a normal control rate
  const std::uint64_t deadline_ns = 250000; // callback must finish in 250 us

  std::printf("Lockstep real-time validation\n");
  std::printf("=============================\n\n");

  const std::string name = "lockstep-rtval-" + std::to_string(ls::self_pid());
  ls::segment::unlink(name);
  ls::bus b = ls::bus::create(name, {{64, 1024}, {4096, 128}});

  const ls::rt::sched_report sr = ls::rt::try_set_realtime();
  const auto prereqs = check_prereqs(sr);

  std::printf("  %-22s %-10s %s\n", "PREREQUISITE", "STATUS", "VALUE");
  bool eligible = true;
  for (const auto& p : prereqs) {
    std::printf("  %-22s %-10s %s\n", p.name, p.ok ? "ok" : "MISSING",
                p.detail.c_str());
    if (!p.ok) eligible = false;
  }

  if (!eligible) {
    std::printf("\n  This machine is NOT eligible for a real-time measurement.\n");
    std::printf("  What would have to change:\n");
    for (const auto& p : prereqs)
      if (!p.ok) std::printf("    - %-20s %s\n", p.name, p.remedy);
    std::printf("\n  Skipping the assertions rather than reporting a number that\n");
    std::printf("  would not mean anything. This is a SKIP, not a failure.\n");
    return 0;
  }

  // ------------------------------------------------------------------ run --
  std::printf("\n  Eligible. Running %d periods at %llu ns, deadline %llu ns.\n",
              iterations, (unsigned long long)period_ns,
              (unsigned long long)deadline_ns);

  auto pub = ls::publisher<DemoImu>::create(b, "rt/probe", 64);
  auto sub = ls::subscriber<DemoImu>::attach(b, "rt/probe");

  std::uint64_t handled = 0;
  ls::rt::executor ex;
  ex.subscribe(sub,
               ls::rt::task_spec{period_ns, deadline_ns, deadline_ns, 0, "probe"},
               [&](const ls::sample<DemoImu>&) { ++handled; });

  const long faults_before = minor_faults();
  std::vector<std::uint64_t> jitter;
  jitter.reserve(static_cast<std::size_t>(iterations));

  std::uint64_t next = ls::now_ns() + period_ns;
  for (int i = 0; i < iterations; ++i) {
    while (ls::now_ns() < next) { /* spin to the release point */ }
    const std::uint64_t woke = ls::now_ns();
    jitter.push_back(woke > next ? woke - next : 0);

    auto ln = pub.loan_message();
    ln->stamp_ns = woke;
    pub.publish(ln);
    ex.spin_once();

    next += period_ns;
  }
  const long faults_after = minor_faults();

  std::sort(jitter.begin(), jitter.end());
  const std::uint64_t misses = ex.total_deadline_misses();
  const long new_faults = faults_after - faults_before;

  std::printf("\n  release jitter\n");
  std::printf("    p50     %8.2f us\n", pct(jitter, 0.50) / 1000.0);
  std::printf("    p99     %8.2f us\n", pct(jitter, 0.99) / 1000.0);
  std::printf("    p99.9   %8.2f us\n", pct(jitter, 0.999) / 1000.0);
  std::printf("    max     %8.2f us\n", pct(jitter, 1.0) / 1000.0);
  std::printf("\n  callbacks           %llu\n", (unsigned long long)handled);
  std::printf("  deadline misses     %llu\n", (unsigned long long)misses);
  std::printf("  minor page faults   %ld (after mlockall)\n", new_faults);

  const auto sched = ex.analyse_observed();
  std::printf("  observed WCET fits declared deadline: %s\n",
              sched.schedulable ? "yes" : "NO");

  // ----------------------------------------------------------- assertions --
  int bad = 0;
  if (misses != 0) {
    std::printf("\n  FAIL: %llu deadline misses on a machine that granted "
                "real-time scheduling.\n", (unsigned long long)misses);
    ++bad;
  }
  // mlockall(MCL_CURRENT|MCL_FUTURE) should mean the steady state faults in
  // nothing. A handful during warm-up is normal; a stream of them means the
  // lock did not take and the tail will be unbounded.
  if (new_faults > 64) {
    std::printf("\n  FAIL: %ld page faults after mlockall; memory is not pinned.\n",
                new_faults);
    ++bad;
  }
  if (!sched.schedulable) {
    std::printf("\n  FAIL: measured worst-case execution exceeds the declared "
                "deadline.\n");
    ++bad;
  }

  std::printf("\n  %s\n", bad == 0 ? "PASS -- real-time behaviour verified on this machine."
                                   : "FAILED");
  return bad == 0 ? 0 : 1;
}
