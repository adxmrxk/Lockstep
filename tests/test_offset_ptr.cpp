#include "lockstep/core/offset_ptr.hpp"

#include <cstdint>
#include <type_traits>

#include "support/block.hpp"
#include "support/check.hpp"

using ls::offset_ptr;

namespace {

struct Node {
  std::int32_t value;
  std::int32_t pad;
  offset_ptr<Node> next;
};

// All targets live inside one opaque block, mirroring how the arena hands out
// storage. Pointing an offset_ptr at a stack local is not supported -- see
// test_provenance.cpp for why.
struct Fixture {
  offset_ptr<std::int32_t> p;
  offset_ptr<std::int32_t> q;
  std::int32_t x;
  std::int32_t y;
  std::int32_t arr[5];
  Node nodes[2];
};

void basics() {
  ls::test::block<Fixture> f;
  f->x = 42;
  f->y = 7;

  LS_CHECK(f->p == nullptr);
  LS_CHECK(!static_cast<bool>(f->p));
  LS_CHECK(f->p.get() == nullptr);
  LS_CHECK_EQ(f->p.raw_offset(), offset_ptr<std::int32_t>::null_offset);

  f->p = &f->x;
  LS_CHECK(static_cast<bool>(f->p));
  LS_CHECK(f->p.get() == &f->x);
  LS_CHECK_EQ(*f->p, 42);

  *f->p = 43;
  LS_CHECK_EQ(f->x, 43);

  f->p = &f->y;
  LS_CHECK(f->p.get() == &f->y);
  LS_CHECK_EQ(*f->p, 7);

  f->p = nullptr;
  LS_CHECK(f->p == nullptr);
}

void arithmetic() {
  ls::test::block<Fixture> f;
  for (std::int32_t i = 0; i < 5; ++i) f->arr[i] = i;

  f->p = f->arr;
  LS_CHECK_EQ(f->p[3], 3);

  f->p += 2;
  LS_CHECK_EQ(*f->p, 2);
  ++f->p;
  LS_CHECK_EQ(*f->p, 3);
  f->p -= 3;
  LS_CHECK_EQ(*f->p, 0);
  LS_CHECK(f->p.get() == f->arr);
}

void comparison() {
  ls::test::block<Fixture> f;
  f->x = 1;
  f->y = 2;

  f->p = &f->x;
  f->q = &f->x;

  // p and q sit at different addresses, so they hold different raw offsets
  // while naming the same target. Comparison must resolve, not compare offsets.
  LS_CHECK(f->p.raw_offset() != f->q.raw_offset());
  LS_CHECK(f->p == f->q);

  f->q = &f->y;
  LS_CHECK(f->p != f->q);
}

void arrow_and_chaining() {
  ls::test::block<Fixture> f;
  f->nodes[0].value = 1;
  f->nodes[1].value = 2;
  f->nodes[0].next = &f->nodes[1];
  f->nodes[1].next = nullptr;

  offset_ptr<Node> head = &f->nodes[0];
  LS_CHECK_EQ(head->value, 1);
  LS_CHECK_EQ(head->next->value, 2);
  LS_CHECK(head->next->next == nullptr);
}

void const_conversion() {
  ls::test::block<Fixture> f;
  f->x = 5;
  f->p = &f->x;

  offset_ptr<const std::int32_t> c = f->p;
  LS_CHECK(c.get() == &f->x);
  LS_CHECK_EQ(*c, 5);
}

}  // namespace

int main() {
  static_assert(std::is_trivially_copyable_v<offset_ptr<int>>);
  static_assert(std::is_standard_layout_v<offset_ptr<int>>);
  static_assert(std::is_trivially_destructible_v<offset_ptr<int>>);
  static_assert(sizeof(offset_ptr<int>) == 8);
  static_assert(alignof(offset_ptr<int>) == 8);

  basics();
  arithmetic();
  comparison();
  arrow_and_chaining();
  const_conversion();
  return ls::test::summary("offset_ptr");
}
