// Negative compile test: std::string owns a heap pointer from the publisher's
// address space. CTest requires this build to FAIL.
#include <cstdint>
#include <string>

#include "lockstep/core/message.hpp"

struct HasStdString {
  std::uint64_t stamp_ns;
  std::string frame_id;
};
LOCKSTEP_MESSAGE(HasStdString, stamp_ns, frame_id);

int main() { return 0; }
