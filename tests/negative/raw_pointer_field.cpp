// Negative compile test: a message with a raw pointer member must be rejected.
// CTest builds this target and requires the build to FAIL.
#include <cstdint>

#include "lockstep/core/message.hpp"

struct HasRawPointer {
  std::uint64_t stamp_ns;
  const std::uint8_t* pixels;  // meaningless in another address space
  std::uint64_t length;
};
LOCKSTEP_MESSAGE(HasRawPointer, stamp_ns, pixels, length);

int main() { return 0; }
