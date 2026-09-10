// core/atomic_ops.hpp : relaxed atomic access to fields that are not atomics.
//
// Lock-free structures here deliberately let one thread write a field while
// another reads it, and sort out afterwards whether what was read was coherent
// -- that is what a seqlock is, and what a Treiber stack's speculative link
// read is. The ALGORITHM is sound. The plain C++ accesses are not: under the
// memory model they are a data race and therefore undefined behaviour, so the
// optimizer is within its rights to do something surprising with them, and
// ThreadSanitizer reports them.
//
// std::atomic_ref with relaxed ordering closes that hole for free. The access
// becomes race-free by definition, and on x86-64 and AArch64 a relaxed load or
// store of a naturally aligned word compiles to the same single instruction the
// plain access would have. No ordering is requested because none is wanted --
// the happens-before edges come from the surrounding acquire/release atomics.
#pragma once

#include <atomic>

namespace ls::detail {

template <class T>
inline void relaxed_store(T& dst, T value) noexcept {
  std::atomic_ref<T>(dst).store(value, std::memory_order_relaxed);
}

template <class T>
inline T relaxed_load(const T& src) noexcept {
  return std::atomic_ref<T>(const_cast<T&>(src)).load(std::memory_order_relaxed);
}

}  // namespace ls::detail
