// Regression test for the miscompile that cost the most to track down.
//
// Reconstructing a pointer from a self-relative offset is pointer arithmetic
// whose base is a known object: the offset_ptr itself, 8 bytes wide. A compiler
// may assume such arithmetic stays inside that object, and therefore that a
// store through the result cannot alias anything else. MSVC 19.29 at /O2 did
// exactly that, and the cross_object_* cases below failed before the fix.
//
// What did NOT fix it (all measured, none of them guesses):
//   * __declspec(noinline) on the encode, so the target address is passed to an
//     opaque callee -- which rules out escape analysis of the target as the cause
//   * storing the target address through a volatile file-scope pointer
//   * _ReadWriteBarrier(), std::atomic_signal_fence(acq_rel)
//   * char* arithmetic instead of an intptr_t round trip
//
// What did fix it: laundering the BASE of the arithmetic, so the compiler stops
// attributing the result to the base object. std::launder, a noinline identity
// on the base, and a volatile round trip on the base all work; std::launder is
// the one that emits no instruction, so that is what offset_ptr uses.
//
// These tests are only meaningful in an optimized build. Keep them passing at
// /O2 and -O2.
#include <cstdint>
#include <cstring>

#include "lockstep/core/offset_ptr.hpp"
#include "support/block.hpp"
#include "support/check.hpp"

namespace {

struct Frame {
  ls::offset_ptr<std::int32_t> cursor;
  std::int32_t values[8];
  std::int64_t checksum;
};

// ---------------------------------------------------------------------------
// The case the fix is for: pointer and target in SEPARATE objects, both fully
// visible to the optimizer. This is not how the bus uses offset_ptr, but it is
// where the miscompile was observable, so it is the sharpest regression test.
// ---------------------------------------------------------------------------

void cross_object_store_is_observable() {
  std::int32_t x = 42;
  ls::offset_ptr<std::int32_t> p = &x;

  LS_CHECK(p.get() == &x);
  LS_CHECK_EQ(*p, 42);

  *p = 43;
  LS_CHECK_EQ(x, 43);  // failed at /O2 before the launder
}

void cross_object_load_sees_direct_writes() {
  std::int32_t x = 1;
  ls::offset_ptr<std::int32_t> p = &x;

  x = 2;
  LS_CHECK_EQ(*p, 2);

  *p = 3;
  LS_CHECK_EQ(x, 3);

  x = 4;
  LS_CHECK_EQ(*p, 4);
}

void cross_object_alternating_access() {
  std::int32_t a = 0;
  std::int32_t b = 0;
  ls::offset_ptr<std::int32_t> pa = &a;
  ls::offset_ptr<std::int32_t> pb = &b;

  for (std::int32_t i = 1; i <= 4; ++i) {
    *pa = i;
    *pb = i * 10;
    LS_CHECK_EQ(a, i);
    LS_CHECK_EQ(b, i * 10);
  }
}

// ---------------------------------------------------------------------------
// The case the bus actually produces: pointer and target inside one block.
// This worked even before the fix -- the arithmetic never left the object --
// but it must keep working.
// ---------------------------------------------------------------------------

void same_block_store_is_observable() {
  ls::test::block<Frame> b;

  b->values[3] = 42;
  b->cursor = &b->values[3];

  LS_CHECK(b->cursor.get() == &b->values[3]);
  LS_CHECK_EQ(*b->cursor, 42);

  *b->cursor = 43;
  LS_CHECK_EQ(b->values[3], 43);

  b->values[3] = 44;
  LS_CHECK_EQ(*b->cursor, 44);
}

void aliasing_holds_after_relocation() {
  ls::test::block<Frame> src(0);
  ls::test::block<Frame> dst(41);

  src->values[1] = 7;
  src->cursor = &src->values[1];

  std::memcpy(dst.get(), src.get(), sizeof(Frame));
  LS_CHECK(dst.get() != src.get());

  // The relocated pointer must address the copy, and stores through it must be
  // visible in the copy and invisible in the original.
  LS_CHECK(dst->cursor.get() == &dst->values[1]);
  *dst->cursor = 99;
  LS_CHECK_EQ(dst->values[1], 99);
  LS_CHECK_EQ(src->values[1], 7);
}

void offsets_are_position_independent() {
  ls::test::block<Frame> a(0);
  ls::test::block<Frame> b(137);

  a->cursor = &a->values[5];
  b->cursor = &b->values[5];

  // Same field, same target index, two different addresses: the stored offset
  // must be identical or the bytes would not be relocatable.
  LS_CHECK_EQ(a->cursor.raw_offset(), b->cursor.raw_offset());
  LS_CHECK(a->cursor.get() != b->cursor.get());
}

}  // namespace

int main() {
  cross_object_store_is_observable();
  cross_object_load_sees_direct_writes();
  cross_object_alternating_access();
  same_block_store_is_observable();
  aliasing_holds_after_relocation();
  offsets_are_position_independent();
  return ls::test::summary("provenance");
}
