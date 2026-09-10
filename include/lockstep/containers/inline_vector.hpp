// containers/inline_vector.hpp : fixed-capacity vector stored inline.
//
// The default container for message fields. Storage lives inside the struct, so
// there are no pointers at all and relocation is free. Use shm_span instead
// when the payload is large enough that it belongs in the arena rather than in
// the message slot (a camera frame, a point cloud).
#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>

#include "lockstep/core/relocatable.hpp"

namespace ls {

template <class T, std::size_t N>
struct inline_vector {
  static_assert(N > 0, "inline_vector needs a non-zero capacity");
  static_assert(is_relocatable_v<T>,
                "inline_vector element type must be relocatable");

  using value_type = T;
  using iterator = T*;
  using const_iterator = const T*;

  // Public to keep the type standard-layout. Treat as private; use the
  // accessors. Both are value-initialized so that unused capacity is a
  // deterministic zero rather than indeterminate bytes -- replay hashes cover
  // the whole slot, not just the live prefix.
  std::uint32_t size_ = 0;
  T data_[N] = {};

  constexpr std::size_t size() const noexcept { return size_; }
  static constexpr std::size_t capacity() noexcept { return N; }
  constexpr bool empty() const noexcept { return size_ == 0; }
  constexpr bool full() const noexcept { return size_ == N; }

  T* data() noexcept { return data_; }
  const T* data() const noexcept { return data_; }

  iterator begin() noexcept { return data_; }
  iterator end() noexcept { return data_ + size_; }
  const_iterator begin() const noexcept { return data_; }
  const_iterator end() const noexcept { return data_ + size_; }

  T& operator[](std::size_t i) noexcept {
    assert(i < size_ && "inline_vector index out of range");
    return data_[i];
  }
  const T& operator[](std::size_t i) const noexcept {
    assert(i < size_ && "inline_vector index out of range");
    return data_[i];
  }

  T& back() noexcept {
    assert(size_ > 0);
    return data_[size_ - 1];
  }
  const T& back() const noexcept {
    assert(size_ > 0);
    return data_[size_ - 1];
  }

  // Returns false instead of throwing: the bus never unwinds on the hot path.
  bool try_push_back(const T& v) noexcept {
    if (full()) return false;
    data_[size_++] = v;
    return true;
  }

  void push_back(const T& v) noexcept {
    const bool ok = try_push_back(v);
    assert(ok && "inline_vector overflow");
    (void)ok;
  }

  void pop_back() noexcept {
    assert(size_ > 0);
    data_[--size_] = T{};
  }

  // Clears back to zeroed storage, not just a reset length, so a recycled slot
  // cannot leak the previous message's bytes into a replay hash.
  void clear() noexcept {
    for (std::size_t i = 0; i < size_; ++i) data_[i] = T{};
    size_ = 0;
  }

  bool resize(std::size_t n) noexcept {
    if (n > N) return false;
    for (std::size_t i = n; i < size_; ++i) data_[i] = T{};
    size_ = static_cast<std::uint32_t>(n);
    return true;
  }
};

template <class T, std::size_t N>
struct is_relocatable<inline_vector<T, N>> : is_relocatable<T> {};

// size_ + N elements, each of which may carry padding of its own.
template <class T, std::size_t N>
struct padding_bytes<inline_vector<T, N>>
    : std::integral_constant<std::size_t,
                             sizeof(inline_vector<T, N>) -
                                 (sizeof(std::uint32_t) + N * useful_bytes_v<T>)> {};

}  // namespace ls
