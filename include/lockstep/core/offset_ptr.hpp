// Lockstep -- deterministic zero-copy robot message bus
// core/offset_ptr.hpp : self-relative pointer, valid at any mapping address.
#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <new>
#include <type_traits>

namespace ls {
namespace detail {

// Reconstructing a pointer from a stored offset is pointer arithmetic whose
// base is a known object -- the offset_ptr itself, 8 bytes wide. A compiler is
// entitled to assume such arithmetic stays inside that object, and therefore
// that a store through the result cannot alias anything else. MSVC 19.29 /O2
// does exactly this:
//
//     int x = 42;
//     offset_ptr<int> p = &x;      // p and x are separate objects
//     *p = 43;
//     assert(x == 43);             // fails: the read is served from a cached 42
//
// This is NOT escape analysis of the target. Passing the target's address to a
// __declspec(noinline) callee, storing it through a volatile global, and
// inserting _ReadWriteBarrier / atomic_signal_fence all leave the bug in place.
// What fixes it is laundering the BASE, so the compiler stops attributing the
// result to the base object. std::launder does that, and is the only candidate
// that costs nothing at runtime -- it emits no instruction and only blocks the
// provenance inference. Measured against noinline-identity, a volatile round
// trip, and a noinline decode, all of which also work but each cost a call or a
// store on every dereference.
//
// tests/test_provenance.cpp pins this down, including the cross-object case
// that miscompiles without the launder.
template <class T>
inline T* launder_offset(void* base, std::int64_t offset) noexcept {
  return std::launder(reinterpret_cast<T*>(static_cast<char*>(base) + offset));
}

}  // namespace detail

// A pointer stored as a signed byte offset from its own address.
//
// Why: /dev/shm segments map at a different base address in every process, so a
// raw pointer written by the publisher is garbage to the subscriber. A
// self-relative pointer survives, because both the pointer and its target move
// by the same delta when the enclosing block is relocated.
//
// ---------------------------------------------------------------------------
// INVARIANT: the pointer and its target must live in the same relocatable block.
// ---------------------------------------------------------------------------
// This is a semantic requirement, not a codegen one. Relocation is defined as
// copying a block's bytes elsewhere; an offset that reaches outside the block
// being copied will, in the destination, point at whatever happens to sit at
// that address. The arena enforces it in practice: it is the only thing that
// hands out storage for offset_ptr targets, and it hands out storage inside the
// segment.
//
// The separate codegen hazard that used to make cross-object use miscompile is
// fixed in detail::launder_offset above, so violating this invariant now
// produces a wrong answer rather than an unpredictable one.
//
// ---------------------------------------------------------------------------
// RELOCATION SEMANTICS -- read this before using it.
// ---------------------------------------------------------------------------
//   Copy/assign are DEFAULTED, so offset_ptr is trivially copyable and the raw
//   offset bytes propagate verbatim. That makes block relocation work:
//
//       std::memcpy(dst_block, src_block, block_size);   // OK: all links intact
//
//   It also makes lifting a single offset_ptr out of its block a bug:
//
//       offset_ptr<T> stray = block->child;              // BUG: dangles
//
//   This is the opposite of boost::interprocess::offset_ptr, which recomputes on
//   copy. We choose trivial copyability deliberately: the bus must be able to
//   static_assert(std::is_trivially_copyable_v<Msg>) and treat a message as
//   bytes. Escape to a raw pointer with .get() when you need to leave the block.
template <class T>
class offset_ptr {
 public:
  using element_type = T;

  // Offset 1 encodes null. A real target one byte past the pointer's own
  // address is impossible for any object we can address, so the value is free.
  static constexpr std::int64_t null_offset = 1;

  constexpr offset_ptr() noexcept : offset_(null_offset) {}
  constexpr offset_ptr(std::nullptr_t) noexcept : offset_(null_offset) {}
  offset_ptr(T* p) noexcept { reset(p); }

  offset_ptr(const offset_ptr&) noexcept = default;
  offset_ptr& operator=(const offset_ptr&) noexcept = default;
  ~offset_ptr() = default;

  offset_ptr& operator=(T* p) noexcept {
    reset(p);
    return *this;
  }
  offset_ptr& operator=(std::nullptr_t) noexcept {
    offset_ = null_offset;
    return *this;
  }

  void reset(T* p) noexcept {
    if (p == nullptr) {
      offset_ = null_offset;
      return;
    }
    // Byte-pointer arithmetic rather than an intptr_t round trip: both are
    // formally UB across unrelated objects, but this form keeps the result
    // derived from a pointer rather than an integer, which is the weaker demand
    // on the optimizer.
    const auto self = reinterpret_cast<const char*>(this);
    const auto target = reinterpret_cast<const char*>(p);
    offset_ = static_cast<std::int64_t>(target - self);
    assert(offset_ != null_offset && "target address aliases the null sentinel");
  }

  T* get() const noexcept {
    if (offset_ == null_offset) return nullptr;
    return detail::launder_offset<T>(const_cast<offset_ptr*>(this), offset_);
  }

  T& operator*() const noexcept {
    assert(offset_ != null_offset && "dereferenced a null offset_ptr");
    return *get();
  }
  T* operator->() const noexcept {
    assert(offset_ != null_offset && "dereferenced a null offset_ptr");
    return get();
  }
  T& operator[](std::ptrdiff_t i) const noexcept { return get()[i]; }

  explicit operator bool() const noexcept { return offset_ != null_offset; }

  offset_ptr& operator+=(std::ptrdiff_t n) noexcept {
    reset(get() + n);
    return *this;
  }
  offset_ptr& operator-=(std::ptrdiff_t n) noexcept { return *this += -n; }
  offset_ptr& operator++() noexcept { return *this += 1; }
  offset_ptr& operator--() noexcept { return *this += -1; }

  // Exposed for tests and for the wire-format dumper; not part of normal use.
  std::int64_t raw_offset() const noexcept { return offset_; }

  // Widening conversion to a const view of the same target.
  operator offset_ptr<const T>() const noexcept {
    return offset_ptr<const T>(static_cast<const T*>(get()));
  }

 public:
  // Public so the class stays standard-layout when nested in message structs.
  // Treat it as private; it is meaningless outside its own storage location.
  std::int64_t offset_;
};

template <class T, class U>
bool operator==(const offset_ptr<T>& a, const offset_ptr<U>& b) noexcept {
  return a.get() == b.get();
}
template <class T, class U>
bool operator!=(const offset_ptr<T>& a, const offset_ptr<U>& b) noexcept {
  return !(a == b);
}
template <class T>
bool operator==(const offset_ptr<T>& a, std::nullptr_t) noexcept {
  return a.get() == nullptr;
}
template <class T>
bool operator!=(const offset_ptr<T>& a, std::nullptr_t) noexcept {
  return a.get() != nullptr;
}

static_assert(sizeof(offset_ptr<int>) == 8, "offset_ptr must be exactly 8 bytes");
static_assert(std::is_trivially_copyable_v<offset_ptr<int>>,
              "offset_ptr must be trivially copyable for block relocation");
static_assert(std::is_standard_layout_v<offset_ptr<int>>,
              "offset_ptr must be standard layout for offsetof() in message traits");

}  // namespace ls
