// An opaque, aligned heap allocation standing in for a mapped shared-memory
// segment.
//
// Tests must not point an offset_ptr at a stack local: the optimizer can prove
// such a target never escapes and is then entitled to assume a store through a
// reconstructed pointer does not alias it. See the invariant block in
// core/offset_ptr.hpp and the demonstration in test_provenance.cpp. Every test
// target lives in one of these instead, which is what the arena will provide in
// phase 2.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

namespace ls::test {

template <class T>
class block {
 public:
  // `slack` shifts the object within the allocation so two blocks land at
  // different addresses and different page offsets.
  explicit block(std::size_t slack = 0)
      : storage_(std::make_unique<std::byte[]>(sizeof(T) + slack + alignof(T))) {
    auto raw = reinterpret_cast<std::uintptr_t>(storage_.get()) + slack;
    const auto mask = static_cast<std::uintptr_t>(alignof(T)) - 1;
    raw = (raw + mask) & ~mask;
    obj_ = ::new (reinterpret_cast<void*>(raw)) T{};
  }

  T* operator->() const noexcept { return obj_; }
  T& operator*() const noexcept { return *obj_; }
  T* get() const noexcept { return obj_; }

 private:
  std::unique_ptr<std::byte[]> storage_;
  T* obj_;
};

}  // namespace ls::test
