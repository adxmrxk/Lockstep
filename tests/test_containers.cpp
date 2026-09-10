#include <array>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include "lockstep/containers/inline_string.hpp"
#include "lockstep/containers/inline_vector.hpp"
#include "lockstep/containers/shm_span.hpp"
#include "support/check.hpp"

namespace {

void vector_basics() {
  ls::inline_vector<std::uint32_t, 4> v{};
  LS_CHECK(v.empty());
  LS_CHECK_EQ(v.capacity(), 4u);

  LS_CHECK(v.try_push_back(10));
  LS_CHECK(v.try_push_back(20));
  LS_CHECK(v.try_push_back(30));
  LS_CHECK(v.try_push_back(40));
  LS_CHECK(v.full());

  // Overflow returns false rather than throwing: the bus never unwinds.
  LS_CHECK(!v.try_push_back(50));
  LS_CHECK_EQ(v.size(), 4u);

  LS_CHECK_EQ(v[0], 10u);
  LS_CHECK_EQ(v.back(), 40u);

  std::uint32_t sum = 0;
  for (auto x : v) sum += x;
  LS_CHECK_EQ(sum, 100u);

  v.pop_back();
  LS_CHECK_EQ(v.size(), 3u);
  LS_CHECK_EQ(v.data_[3], 0u);  // popped slot was zeroed, not just orphaned
}

void vector_clear_zeroes_storage() {
  ls::inline_vector<std::uint32_t, 4> v{};
  v.push_back(0xAAAAAAAA);
  v.push_back(0xBBBBBBBB);
  v.clear();

  LS_CHECK(v.empty());
  // Recycled slots must not leak the previous message's bytes, or a replay
  // hash over the whole slot would not reproduce.
  for (std::size_t i = 0; i < v.capacity(); ++i) LS_CHECK_EQ(v.data_[i], 0u);
}

void vector_resize() {
  ls::inline_vector<std::uint32_t, 4> v{};
  v.push_back(1);
  v.push_back(2);
  v.push_back(3);

  LS_CHECK(v.resize(1));
  LS_CHECK_EQ(v.size(), 1u);
  LS_CHECK_EQ(v.data_[1], 0u);
  LS_CHECK_EQ(v.data_[2], 0u);
  LS_CHECK(!v.resize(9));
  LS_CHECK_EQ(v.size(), 1u);
}

void string_basics() {
  ls::inline_string<8> s{};
  LS_CHECK(s.empty());

  LS_CHECK(s.assign("imu0"));
  LS_CHECK_EQ(s.size(), 4u);
  LS_CHECK(s.view() == std::string_view("imu0"));
  LS_CHECK(s == std::string_view("imu0"));

  // Refuses to truncate silently.
  LS_CHECK(!s.assign("a_very_long_frame_id"));
  LS_CHECK(s == std::string_view("imu0"));

  LS_CHECK(s.assign("12345678"));  // exactly capacity, no NUL required
  LS_CHECK_EQ(s.size(), 8u);

  s.clear();
  LS_CHECK(s.empty());
  for (std::size_t i = 0; i < s.capacity(); ++i) LS_CHECK_EQ(s.data_[i], '\0');
}

void string_assign_zeroes_tail() {
  ls::inline_string<8> s{};
  s.assign("abcdefgh");
  s.assign("xy");
  LS_CHECK_EQ(s.size(), 2u);
  for (std::size_t i = 2; i < s.capacity(); ++i) LS_CHECK_EQ(s.data_[i], '\0');
}

void span_basics() {
  std::uint8_t buf[16] = {};
  for (std::size_t i = 0; i < 16; ++i) buf[i] = static_cast<std::uint8_t>(i);

  ls::shm_span<std::uint8_t> s{};
  LS_CHECK(s.empty());
  LS_CHECK(s.data() == nullptr);

  s.bind(buf, 16);
  LS_CHECK_EQ(s.size(), 16u);
  LS_CHECK_EQ(s.size_bytes(), 16u);
  LS_CHECK_EQ(s[7], 7u);

  std::size_t n = 0;
  for (auto b : s) n += b;
  LS_CHECK_EQ(n, 120u);

  s.reset();
  LS_CHECK(s.empty());
  LS_CHECK(s.data() == nullptr);
}

}  // namespace

int main() {
  using V = ls::inline_vector<std::uint32_t, 4>;
  using S = ls::inline_string<8>;
  using P = ls::shm_span<std::uint8_t>;

  static_assert(ls::is_relocatable_v<V>);
  static_assert(ls::is_relocatable_v<S>);
  static_assert(ls::is_relocatable_v<P>);
  static_assert(std::is_trivially_copyable_v<V>);
  static_assert(std::is_trivially_copyable_v<S>);
  static_assert(std::is_trivially_copyable_v<P>);
  static_assert(std::is_standard_layout_v<V>);
  static_assert(std::is_standard_layout_v<S>);
  static_assert(std::is_standard_layout_v<P>);

  // Relocatability propagates through arrays and nesting...
  static_assert(ls::is_relocatable_v<V[2]>);
  static_assert(ls::is_relocatable_v<std::array<S, 3>>);
  static_assert(ls::is_relocatable_v<ls::inline_vector<P, 2>>);

  // ...and a raw pointer is rejected wherever it shows up.
  static_assert(!ls::is_relocatable_v<int*>);
  static_assert(!ls::is_relocatable_v<const char*>);
  static_assert(!ls::is_relocatable_v<V*>);

  vector_basics();
  vector_clear_zeroes_storage();
  vector_resize();
  string_basics();
  string_assign_zeroes_tail();
  span_basics();
  return ls::test::summary("containers");
}
