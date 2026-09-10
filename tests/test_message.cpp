#include <cstdint>
#include <string_view>

#include "lockstep/containers/inline_string.hpp"
#include "lockstep/containers/inline_vector.hpp"
#include "lockstep/containers/shm_span.hpp"
#include "lockstep/core/message.hpp"
#include "support/check.hpp"

struct ImuSample {
  std::uint64_t stamp_ns;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};
LOCKSTEP_MESSAGE(ImuSample, stamp_ns, ax, ay, az, gx, gy, gz);

struct CameraFrame {
  std::uint64_t stamp_ns;
  std::uint32_t width;
  std::uint32_t height;
  ls::inline_string<16> frame_id;
  ls::shm_span<std::uint8_t> pixels;
};
LOCKSTEP_MESSAGE(CameraFrame, stamp_ns, width, height, frame_id, pixels);

struct Tight {
  std::uint32_t a;
  std::uint32_t b;
};
LOCKSTEP_MESSAGE(Tight, a, b);

struct Padded {
  std::uint8_t a;
  std::uint64_t b;
};
LOCKSTEP_MESSAGE(Padded, a, b);

struct Undeclared {
  int x;
};

namespace {

void traits_are_populated() {
  using T = ls::message_traits<ImuSample>;
  LS_CHECK(T::declared);
  LS_CHECK(T::name == std::string_view("ImuSample"));
  LS_CHECK_EQ(T::field_count, 7u);
  LS_CHECK_EQ(T::size, sizeof(ImuSample));
  LS_CHECK(T::layout_hash != 0);

  LS_CHECK(T::fields[0].name == std::string_view("stamp_ns"));
  LS_CHECK_EQ(T::fields[0].offset, 0u);
  LS_CHECK_EQ(T::fields[0].size, 8u);
  LS_CHECK(T::fields[1].name == std::string_view("ax"));
  LS_CHECK_EQ(T::fields[1].offset, 8u);
}

void padding_is_reported() {
  LS_CHECK(!ls::message_traits<Tight>::has_padding);
  LS_CHECK(ls::message_traits<Padded>::has_padding);
  LS_CHECK(!ls::message_traits<ImuSample>::has_padding);
}

void type_names_are_recorded() {
  // Exact spelling is compiler-specific; all we require is that a field's type
  // contributes something non-empty and stable to the hash.
  using T = ls::message_traits<CameraFrame>;
  for (std::size_t i = 0; i < T::field_count; ++i) LS_CHECK(!T::fields[i].type.empty());
}

}  // namespace

int main() {
  // Declared messages satisfy the full bus contract.
  static_assert(ls::Message<ImuSample>);
  static_assert(ls::Message<CameraFrame>);
  static_assert(ls::ZeroCopyable<ImuSample>);
  static_assert(ls::is_relocatable_v<ImuSample>);
  static_assert(ls::is_relocatable_v<CameraFrame>);

  // A struct that was never declared is rejected even though it is a
  // perfectly ordinary POD -- opt-in is the whole point.
  static_assert(!ls::message_traits<Undeclared>::declared);
  static_assert(!ls::Message<Undeclared>);
  static_assert(!ls::is_relocatable_v<Undeclared>);

  traits_are_populated();
  padding_is_reported();
  type_names_are_recorded();
  return ls::test::summary("message");
}
