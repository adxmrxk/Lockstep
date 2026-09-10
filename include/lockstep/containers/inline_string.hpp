// containers/inline_string.hpp : fixed-capacity string stored inline.
//
// std::string cannot cross a process boundary: SSO aside, it owns a heap
// pointer from the publisher's address space. This is the replacement for
// short, bounded text fields (frame ids, sensor names, topic labels).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string_view>

#include "lockstep/core/relocatable.hpp"
#include "lockstep/core/type_tag.hpp"

namespace ls {

template <std::size_t N>
struct inline_string {
  static_assert(N > 0, "inline_string needs a non-zero capacity");

  // Public for standard layout; zeroed so unused bytes stay deterministic.
  std::uint32_t size_ = 0;
  char data_[N] = {};

  constexpr std::size_t size() const noexcept { return size_; }
  static constexpr std::size_t capacity() noexcept { return N; }
  constexpr bool empty() const noexcept { return size_ == 0; }

  const char* data() const noexcept { return data_; }

  std::string_view view() const noexcept {
    return std::string_view(data_, size_);
  }

  // Truncating assign is a silent data bug, so refuse instead and let the
  // caller decide. Returns false and leaves the value unchanged if too long.
  bool assign(std::string_view s) noexcept {
    if (s.size() > N) return false;
    for (std::size_t i = 0; i < s.size(); ++i) data_[i] = s[i];
    for (std::size_t i = s.size(); i < N; ++i) data_[i] = '\0';
    size_ = static_cast<std::uint32_t>(s.size());
    return true;
  }

  void clear() noexcept {
    for (std::size_t i = 0; i < N; ++i) data_[i] = '\0';
    size_ = 0;
  }

  friend bool operator==(const inline_string& a, const inline_string& b) noexcept {
    return a.view() == b.view();
  }
  friend bool operator==(const inline_string& a, std::string_view b) noexcept {
    return a.view() == b;
  }
};

template <std::size_t N>
struct is_relocatable<inline_string<N>> : std::true_type {};

// size_ + data_[N]; anything beyond that is trailing alignment padding.
// Portable identity: shape and capacity, not the compiler's spelling.
template <std::size_t N>
struct type_tag_of<inline_string<N>> {
  static constexpr std::uint64_t value = detail::tag_mix(
      static_cast<std::uint64_t>(N),
      detail::tag_mix(std::string_view("istr"), detail::tag_basis));
};

template <std::size_t N>
struct padding_bytes<inline_string<N>>
    : std::integral_constant<std::size_t,
                             sizeof(inline_string<N>) - (sizeof(std::uint32_t) + N)> {};

}  // namespace ls
