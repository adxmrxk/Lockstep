// A publisher that exists to be killed.
//
// Publishes ImuSample in a tight loop until SIGKILL arrives. Every field is
// derived from one counter, so the parent can tell a coherent message from a
// torn one without knowing when the kill landed.
//
//   argv[1]  segment name
//   argv[2]  microseconds to hold each loan open before committing
//
// The hold is fault injection, not realism. It widens the window between
// "claimed the slot" and "committed it" so that a kill at a random moment
// actually lands inside it often enough to be worth measuring. With no hold the
// window is a few nanoseconds and thousands of kills would almost never hit it.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "lockstep/pubsub.hpp"
#include "support/demo_msgs.hpp"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: crash_publisher_child <segment> <hold-us>\n");
    return 2;
  }
  const char* name = argv[1];
  const auto hold_us = static_cast<unsigned>(std::strtoul(argv[2], nullptr, 10));

  try {
    ls::bus b = ls::bus::open(name);
    auto pub = ls::publisher<ImuSample>::create(b, "imu/crash", 64);

    for (std::uint64_t v = 1;; ++v) {
      auto ln = pub.loan_message();
      const float f = static_cast<float>(v & 0xffffu);
      ln->stamp_ns = v;
      ln->ax = f;
      ln->ay = f;
      ln->az = f;
      ln->gx = f;
      ln->gy = f;
      ln->gz = f;

      // The slot is claimed but not committed for this whole window. A SIGKILL
      // landing here is exactly the case the reaper has to repair.
      if (hold_us > 0) ::usleep(hold_us);

      pub.publish(ln);
    }
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "  [crash-child] threw: %s\n", ex.what());
    return 1;
  }
}
