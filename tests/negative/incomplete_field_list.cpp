// Negative compile test: forgetting a member must be rejected, because the
// omitted bytes would fall outside the layout hash and outside every
// relocation guarantee. CTest requires this build to FAIL.
#include <cstdint>

#include "lockstep/core/message.hpp"

struct MissingAField {
  std::uint32_t a;
  std::uint32_t b;
  std::uint32_t forgotten;
};
LOCKSTEP_MESSAGE(MissingAField, a, b);

int main() { return 0; }
