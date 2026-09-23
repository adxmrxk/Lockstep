// Slab pools: allocation, recycling, exhaustion, size-class selection, and a
// concurrent hammer on the lock-free free list.
#include <atomic>
#include <cstdint>
#include <cstring>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "lockstep/shm/arena.hpp"
#include "lockstep/core/process.hpp"
#include "lockstep/shm/segment.hpp"
#include "support/check.hpp"

namespace {

std::string unique_name(const char* tag) {
  return std::string("lockstep-arena-") + tag + "-" + std::to_string(ls::self_pid());
}

struct fixture {
  ls::segment seg;
  ls::arena a;

  fixture(std::vector<ls::pool_spec> pools, const char* tag) {
    const std::string name = unique_name(tag);
    ls::segment::unlink(name);
    const std::size_t need = 4096 + ls::arena::bytes_required(pools.data(), pools.size());
    seg = ls::segment::create(name, need);
    a = ls::arena::construct(seg.base(), 4096, pools.data(), pools.size());
  }
};

void allocates_from_the_smallest_class_that_fits() {
  fixture f({{64, 4}, {1024, 2}}, "classes");

  const std::uint64_t small = f.a.allocate(32);
  LS_CHECK(small != 0);
  LS_CHECK_EQ(f.a.block_size_of(small), 64u);

  const std::uint64_t big = f.a.allocate(700);
  LS_CHECK(big != 0);
  LS_CHECK_EQ(f.a.block_size_of(big), 1024u);

  // A request larger than every class fails rather than returning something
  // too small.
  LS_CHECK_EQ(f.a.allocate(4096), 0u);
}

void blocks_are_distinct_and_aligned() {
  fixture f({{64, 8}}, "distinct");

  std::set<std::uint64_t> seen;
  for (int i = 0; i < 8; ++i) {
    const std::uint64_t off = f.a.allocate(64);
    LS_CHECK(off != 0);
    LS_CHECK(seen.insert(off).second);          // no block handed out twice
    LS_CHECK_EQ(off % ls::arena_block_align, 0u);  // cache-line aligned
  }
  LS_CHECK_EQ(f.a.allocate(64), 0u);  // exhausted
  LS_CHECK_EQ(f.a.live_blocks(), 8u);
}

void freed_blocks_come_back() {
  fixture f({{64, 2}}, "recycle");

  const std::uint64_t a = f.a.allocate(64);
  const std::uint64_t b = f.a.allocate(64);
  LS_CHECK(a != 0 && b != 0);
  LS_CHECK_EQ(f.a.allocate(64), 0u);

  f.a.deallocate(a);
  LS_CHECK_EQ(f.a.live_blocks(), 1u);

  const std::uint64_t c = f.a.allocate(64);
  LS_CHECK(c != 0);
  LS_CHECK_EQ(c, a);  // LIFO: the block just freed is the one handed back

  f.a.deallocate(b);
  f.a.deallocate(c);
  LS_CHECK_EQ(f.a.live_blocks(), 0u);
}

void blocks_are_writable_for_their_whole_size() {
  fixture f({{256, 4}}, "write");

  const std::uint64_t off = f.a.allocate(256);
  LS_CHECK(off != 0);
  auto* p = f.a.at<std::uint8_t>(off);
  std::memset(p, 0xab, 256);
  LS_CHECK_EQ(p[0], 0xab);
  LS_CHECK_EQ(p[255], 0xab);
  LS_CHECK(f.seg.contains(p, 256));
}

// The free list is a tagged Treiber stack. Hammer it from several threads and
// assert the invariant that actually matters: no block is ever held by two
// allocators at once.
void concurrent_alloc_free_never_double_hands_a_block() {
  constexpr int kThreads = 4;
  constexpr int kRounds = 4000;
  constexpr std::uint64_t kBlocks = 32;

  fixture f({{64, kBlocks}}, "concurrent");

  // One byte of ownership state per block, indexed by block number.
  std::vector<std::atomic<int>> owner(kBlocks);
  for (auto& o : owner) o.store(-1, std::memory_order_relaxed);

  std::atomic<int> collisions{0};
  std::atomic<int> allocated{0};

  const std::uint64_t storage = f.a.pool(0).storage_offset;
  const std::uint64_t bs = f.a.pool(0).block_size;

  std::vector<std::thread> ts;
  for (int t = 0; t < kThreads; ++t) {
    ts.emplace_back([&, t] {
      for (int r = 0; r < kRounds; ++r) {
        const std::uint64_t off = f.a.allocate(64);
        if (off == 0) continue;  // legitimately exhausted right now
        allocated.fetch_add(1, std::memory_order_relaxed);

        const std::uint64_t idx = (off - storage) / bs;
        int expected = -1;
        if (!owner[idx].compare_exchange_strong(expected, t,
                                                std::memory_order_acq_rel)) {
          collisions.fetch_add(1, std::memory_order_relaxed);
        }

        // Scribble, to catch a block being shared through the data path too.
        // This writes the block's first 8 bytes, which is exactly where the
        // free-list link used to live -- and is why ThreadSanitizer flagged
        // this line before the links were moved into their own array.
        auto* p = f.a.at<std::uint64_t>(off);
        *p = static_cast<std::uint64_t>(t);

        owner[idx].store(-1, std::memory_order_release);
        f.a.deallocate(off);
      }
    });
  }
  for (auto& th : ts) th.join();

  LS_CHECK(allocated.load() > 0);
  LS_CHECK_EQ(collisions.load(), 0);
  LS_CHECK_EQ(f.a.live_blocks(), 0u);

  // Every block must still be reachable: drain the pool and count.
  std::uint64_t drained = 0;
  while (f.a.allocate(64) != 0) ++drained;
  LS_CHECK_EQ(drained, kBlocks);
}

}  // namespace

int main() {
  allocates_from_the_smallest_class_that_fits();
  blocks_are_distinct_and_aligned();
  freed_blocks_come_back();
  blocks_are_writable_for_their_whole_size();
  concurrent_alloc_free_never_double_hands_a_block();
  return ls::test::summary("arena");
}
