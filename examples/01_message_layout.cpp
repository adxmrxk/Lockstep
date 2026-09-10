// Prints what the bus knows about a message type at compile time: the field
// table, the layout hash both peers will exchange at connect time, and whether
// the struct carries padding the publisher must zero before it hashes a slot.
#include <cstdint>
#include <cstdio>

#include "lockstep/containers/inline_string.hpp"
#include "lockstep/containers/inline_vector.hpp"
#include "lockstep/containers/shm_span.hpp"
#include "lockstep/core/message.hpp"

struct ImuSample {
  std::uint64_t stamp_ns;
  float ax;
  float ay;
  float az;
};
LOCKSTEP_MESSAGE(ImuSample, stamp_ns, ax, ay, az);

struct CameraFrame {
  std::uint64_t stamp_ns;
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t stride;
  std::uint32_t channels;
  ls::inline_string<16> frame_id;
  ls::shm_span<std::uint8_t> pixels;
};
LOCKSTEP_MESSAGE(CameraFrame, stamp_ns, width, height, stride, channels,
                 frame_id, pixels);

template <class T>
void describe() {
  using M = ls::message_traits<T>;
  std::printf("%.*s\n", static_cast<int>(M::name.size()), M::name.data());
  std::printf("  size %zu  align %zu  fields %zu  padding %s\n", M::size,
              M::align, M::field_count, M::has_padding ? "yes" : "no");
  std::printf("  layout hash 0x%016llx\n",
              static_cast<unsigned long long>(M::layout_hash));
  std::printf("  %-6s %-6s %-14s %s\n", "off", "size", "field", "type");
  for (std::size_t i = 0; i < M::field_count; ++i) {
    const auto& f = M::fields[i];
    std::printf("  %-6zu %-6zu %-14.*s %.*s\n", f.offset, f.size,
                static_cast<int>(f.name.size()), f.name.data(),
                static_cast<int>(f.type.size()), f.type.data());
  }
  std::printf("\n");
}

int main() {
  describe<ImuSample>();
  describe<CameraFrame>();

  std::printf("A CameraFrame message slot is %zu bytes regardless of image "
              "size;\nthe pixels live in the arena and the span points at "
              "them.\n",
              sizeof(CameraFrame));
  return 0;
}
