// containers/shm_span.hpp : a view of a payload living elsewhere in the same
// shared-memory block.
//
// A 4K RGB frame is ~24 MB and has no business sitting inside a fixed-size
// message slot. The frame body is allocated in the segment's arena and the
// message carries a shm_span pointing at it.
//
// INVARIANT: the target must live in the same relocatable block as the span.
// A span into a different segment, or into process-local heap, survives the
// publisher's address space and nothing else. bind() cannot check this -- the
// arena allocator that hands out the storage is what enforces it.
#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>

#include "lockstep/core/offset_ptr.hpp"
#include "lockstep/core/relocatable.hpp"

namespace ls {

template <class T>
struct shm_span {
  static_assert(is_relocatable_v<T>, "shm_span element type must be relocatable");

  using value_type = T;

  // Public for standard layout. Ordered pointer-then-size, and size is 64-bit,
  // so the struct is exactly 16 bytes with no interior padding on any target we
  // support -- padding bytes are indeterminate and would break replay hashing.
  offset_ptr<T> data_;
  std::uint64_t size_ = 0;

  void bind(T* p, std::size_t n) noexcept {
    data_ = p;
    size_ = n;
  }
  void reset() noexcept {
    data_ = nullptr;
    size_ = 0;
  }

  std::size_t size() const noexcept { return static_cast<std::size_t>(size_); }
  bool empty() const noexcept { return size_ == 0; }
  std::size_t size_bytes() const noexcept { return size() * sizeof(T); }

  T* data() noexcept { return data_.get(); }
  const T* data() const noexcept { return data_.get(); }

  T* begin() noexcept { return data_.get(); }
  T* end() noexcept { return data_.get() + size_; }
  const T* begin() const noexcept { return data_.get(); }
  const T* end() const noexcept { return data_.get() + size_; }

  T& operator[](std::size_t i) noexcept {
    assert(i < size_ && "shm_span index out of range");
    return data_.get()[i];
  }
  const T& operator[](std::size_t i) const noexcept {
    assert(i < size_ && "shm_span index out of range");
    return data_.get()[i];
  }
};

template <class T>
struct is_relocatable<shm_span<T>> : is_relocatable<T> {};

// offset_ptr + size_, both 8 bytes: no padding. The element type's own padding
// does not count, because the elements live in the arena, not in this struct.
template <class T>
struct padding_bytes<shm_span<T>>
    : std::integral_constant<std::size_t, sizeof(shm_span<T>) - 16u> {};

static_assert(sizeof(shm_span<std::uint8_t>) == 16,
              "shm_span must be 16 bytes with no interior padding");

}  // namespace ls
