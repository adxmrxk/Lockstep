// The load-bearing test for the whole project.
//
// Builds a linked structure inside one contiguous block, copies the block's raw
// bytes to a different address (which is what mapping a /dev/shm segment into a
// second process amounts to), and checks that every internal link still
// resolves -- against the relocated block, not the original.
#include <cstdint>
#include <cstring>
#include <string_view>
#include <type_traits>

#include "lockstep/containers/inline_string.hpp"
#include "lockstep/containers/inline_vector.hpp"
#include "lockstep/containers/shm_span.hpp"
#include "lockstep/core/offset_ptr.hpp"
#include "support/block.hpp"
#include "support/check.hpp"

namespace {

struct Node {
  std::uint32_t id;
  std::uint32_t pad;
  ls::offset_ptr<Node> next;
};

struct Block {
  ls::inline_string<16> label;
  ls::inline_vector<std::uint32_t, 8> counters;
  ls::offset_ptr<Node> head;
  ls::shm_span<std::uint8_t> payload;
  Node nodes[4];
  std::uint8_t bytes[64];
};

void build(Block& b) {
  b.label.assign("front_camera");
  for (std::uint32_t i = 0; i < 5; ++i) b.counters.push_back(i * 11);

  for (std::uint32_t i = 0; i < 4; ++i) {
    b.nodes[i].id = 100 + i;
    b.nodes[i].pad = 0;
    b.nodes[i].next = (i + 1 < 4) ? &b.nodes[i + 1] : nullptr;
  }
  b.head = &b.nodes[0];

  for (std::size_t i = 0; i < sizeof(b.bytes); ++i)
    b.bytes[i] = static_cast<std::uint8_t>(i * 3);
  b.payload.bind(b.bytes, sizeof(b.bytes));
}

void verify(const Block& b, const char* where) {
  const int before = ls::test::failures;

  LS_CHECK(b.label == std::string_view("front_camera"));
  LS_CHECK_EQ(b.counters.size(), 5u);
  LS_CHECK_EQ(b.counters[4], 44u);

  // Every link must resolve to a target inside THIS block, not the source one.
  const auto lo = reinterpret_cast<std::uintptr_t>(&b);
  const auto hi = lo + sizeof(Block);

  LS_CHECK(b.head != nullptr);
  const auto head_addr = reinterpret_cast<std::uintptr_t>(b.head.get());
  LS_CHECK(head_addr >= lo && head_addr < hi);

  int walked = 0;
  for (const Node* n = b.head.get(); n != nullptr; n = n->next.get()) {
    LS_CHECK_EQ(n->id, static_cast<std::uint32_t>(100 + walked));
    const auto a = reinterpret_cast<std::uintptr_t>(n);
    LS_CHECK(a >= lo && a < hi);
    ++walked;
  }
  LS_CHECK_EQ(walked, 4);

  LS_CHECK_EQ(b.payload.size(), 64u);
  const auto pay = reinterpret_cast<std::uintptr_t>(b.payload.data());
  LS_CHECK(pay >= lo && pay < hi);
  for (std::size_t i = 0; i < b.payload.size(); ++i)
    LS_CHECK_EQ(b.payload[i], static_cast<std::uint8_t>(i * 3));

  if (ls::test::failures != before)
    std::fprintf(stderr, "  (the failures above were observed at: %s)\n", where);
}

void relocate_preserves_links() {
  // Different addresses and different page offsets, as two mappings of the
  // same segment would have.
  ls::test::block<Block> src(0);
  ls::test::block<Block> dst(97);
  ls::test::block<Block> far(4096);

  build(*src);
  verify(*src, "original");

  LS_CHECK(src.get() != dst.get());

  std::memcpy(dst.get(), src.get(), sizeof(Block));
  std::memcpy(far.get(), src.get(), sizeof(Block));

  verify(*dst, "relocated near");
  verify(*far, "relocated far");
  verify(*src, "original after copy");

  // Position independence means the raw bytes are identical: no offset had to
  // be rewritten for the copy to be valid.
  LS_CHECK(std::memcmp(src.get(), dst.get(), sizeof(Block)) == 0);
  LS_CHECK_EQ(src->head.raw_offset(), dst->head.raw_offset());
}

void mutating_a_copy_does_not_touch_the_original() {
  ls::test::block<Block> src(0);
  ls::test::block<Block> dst(31);

  build(*src);
  std::memcpy(dst.get(), src.get(), sizeof(Block));

  dst->nodes[0].id = 999;
  LS_CHECK_EQ(dst->head->id, 999u);
  LS_CHECK_EQ(src->head->id, 100u);
}

}  // namespace

int main() {
  static_assert(std::is_trivially_copyable_v<Block>,
                "a relocatable block must be memcpy-able");
  static_assert(std::is_standard_layout_v<Block>);

  relocate_preserves_links();
  mutating_a_copy_does_not_touch_the_original();
  return ls::test::summary("relocation");
}
