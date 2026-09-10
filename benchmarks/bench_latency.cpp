// End-to-end latency of the bus, as a distribution rather than an average.
//
// READ THIS BEFORE QUOTING A NUMBER FROM IT.
//
// This is measured on whatever machine you run it on. If that is WSL2, a
// container, or any stock kernel, the tail is dominated by the scheduler and
// not by the bus, and the numbers are useful only for spotting a regression
// against themselves. They are NOT a real-time result, and the README's
// position on this is the right one: real numbers need PREEMPT_RT, isolated
// cores, mlockall and CAP_SYS_NICE. The tool prints whether it got any of that.
//
// What is measured: publisher timestamps the message immediately before
// publish(); subscriber timestamps immediately after take() returns it. The
// difference is transport latency including the seqlock handshake. Both ends
// read the same CLOCK_MONOTONIC in the same process, so there is no clock-skew
// correction to get wrong.
//
// Percentiles come from a plain sorted vector rather than HdrHistogram: the
// sample count is small enough that exact order statistics are cheaper than a
// dependency, and exact beats approximate when the whole point is the tail.
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unistd.h>
#include <vector>

#include "demo_msgs.hpp"
#include "lockstep/pubsub.hpp"
#include "lockstep/rt/executor.hpp"

namespace {

std::uint64_t percentile(std::vector<std::uint64_t>& v, double p) {
  if (v.empty()) return 0;
  const std::size_t idx = static_cast<std::size_t>(
      static_cast<double>(v.size() - 1) * p);
  return v[idx];
}

void report(const char* label, std::vector<std::uint64_t> samples,
            std::size_t payload_bytes) {
  std::sort(samples.begin(), samples.end());
  std::uint64_t sum = 0;
  for (auto s : samples) sum += s;

  std::printf("\n  %s  (%zu bytes/message, %zu samples)\n", label, payload_bytes,
              samples.size());
  std::printf("    mean    %8.2f us\n",
              static_cast<double>(sum) / static_cast<double>(samples.size()) / 1000.0);
  std::printf("    min     %8.2f us\n", percentile(samples, 0.0) / 1000.0);
  std::printf("    p50     %8.2f us\n", percentile(samples, 0.50) / 1000.0);
  std::printf("    p90     %8.2f us\n", percentile(samples, 0.90) / 1000.0);
  std::printf("    p99     %8.2f us\n", percentile(samples, 0.99) / 1000.0);
  std::printf("    p99.9   %8.2f us\n", percentile(samples, 0.999) / 1000.0);
  std::printf("    max     %8.2f us\n", percentile(samples, 1.0) / 1000.0);
}

// `capacity` is the ring depth. It has to shrink as the payload grows: every
// slot owns its body block for the life of the bus, so 1024 slots of 1 MB would
// be a gigabyte of arena reserved up front. That preallocation is what keeps the
// publish path allocation-free, and this is its price.
template <class T>
std::vector<std::uint64_t> measure(ls::bus& b, const char* topic, int iterations,
                                   std::size_t body_bytes, std::size_t capacity) {
  auto pub = ls::publisher<T>::create(b, topic, capacity, body_bytes);
  auto sub = ls::subscriber<T>::attach(b, topic);

  std::vector<std::uint64_t> samples;
  samples.reserve(static_cast<std::size_t>(iterations));

  for (int i = 0; i < iterations; ++i) {
    auto ln = pub.loan_message();
    if (body_bytes > 0) {
      // Touch the payload: a transport that copies would copy this, and the
      // whole claim is that this one does not.
      std::memset(ln.body(), i & 0xff, body_bytes);
    }
    ln->stamp_ns = ls::now_ns();
    pub.publish(ln);

    ls::sample<T> s;
    for (;;) {
      const auto rr = sub.take(s);
      if (rr == ls::read_result::ok) {
        samples.push_back(ls::now_ns() - s->stamp_ns);
        break;
      }
      if (rr == ls::read_result::retry) continue;
      if (rr == ls::read_result::empty) continue;
      break;  // overrun or abandoned: skip this one
    }
  }
  return samples;
}

}  // namespace

int main(int argc, char** argv) {
  const int iterations = argc > 1 ? std::atoi(argv[1]) : 100000;

  std::printf("Lockstep latency\n");
  std::printf("================\n");

  // Map the segment BEFORE asking for mlockall. try_set_realtime uses
  // MCL_FUTURE, which makes every subsequent mmap locked memory, and a
  // multi-megabyte arena then fails with EAGAIN against a stock RLIMIT_MEMLOCK.
  // Memory setup first, real-time setup second -- which is the correct order for
  // a real-time process anyway.
  const std::string name = "lockstep-bench-" + std::to_string(::getpid());
  ls::segment::unlink(name);
  ls::bus b = ls::bus::create(name, {{64, 4096},
                                     {256, 2048},
                                     {4096, 512},
                                     {65536, 128},
                                     {1u << 20, 24}});

  const ls::rt::sched_report sr = ls::rt::try_set_realtime();
  std::printf("\n  scheduling: realtime=%s  mlockall=%s\n",
              sr.got_realtime ? "yes" : "NO", sr.memory_locked ? "yes" : "NO");
  if (!sr.trustworthy_for_timing()) {
    std::printf("  %s\n", sr.detail.c_str());
    std::printf("\n  *** These are NOT real-time numbers. Without SCHED_FIFO on a\n");
    std::printf("  *** PREEMPT_RT kernel with isolated cores, the tail below is the\n");
    std::printf("  *** scheduler, not the bus. Use them to detect regressions against\n");
    std::printf("  *** themselves and for nothing else.\n");
  }

  report("small message, inline",
         measure<DemoImu>(b, "bench/imu", iterations, 0, 1024), sizeof(DemoImu));
  report("64 KB frame, out-of-line",
         measure<DemoFrame>(b, "bench/frame64k", iterations / 10, 64 * 1024, 64),
         64 * 1024);
  report("1 MB frame, out-of-line",
         measure<DemoFrame>(b, "bench/frame1m", iterations / 100, 1u << 20, 16),
         1u << 20);

  std::printf("\n  Note: the message DESCRIPTOR is %zu bytes regardless of payload;\n",
              sizeof(DemoFrame));
  std::printf("  the body stays in the arena and is never copied by the transport.\n");
  return 0;
}
