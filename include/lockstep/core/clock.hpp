// core/clock.hpp : the one place a raw time source is read.
//
// Kept separate and tiny because a wall-clock read is a hidden input, and every
// hidden input is a reason a replay diverges. Anything that wants to be
// replayable reads through ls::deterministic_clock in replay/journal.hpp, not
// through this directly.
#pragma once

#include <chrono>
#include <cstdint>

namespace ls {

inline std::uint64_t now_ns() noexcept {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace ls
