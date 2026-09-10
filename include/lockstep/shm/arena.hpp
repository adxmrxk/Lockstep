// shm/arena.hpp : fixed-size slab pools inside a shared-memory segment.
//
// A message bus cannot call malloc. The publisher's allocator lives in the
// publisher's address space, so a pointer it returns means nothing to the
// subscriber, and a robot that allocates on the hot path has no bound on its
// worst case. So the segment carries its own storage, carved at construction
// into pools of fixed-size blocks, and "allocating" is popping a block index
// off a free list.
//
// Everything here is expressed in segment-relative offsets, never pointers,
// because the segment maps at a different base address in every process.
// Offset 0 is never a valid block -- the segment header lives there -- so 0
// doubles as the failure value.
//
// The free list is a Treiber stack with a 16-bit tag in the high bits of the
// head to defeat ABA. Its `next` links live in a SEPARATE array, not in the
// first bytes of each free block, which is where the obvious implementation
// puts them.
//
// The obvious implementation has a real bug, and ThreadSanitizer found it here.
// A thread in allocate() reads the head, then speculatively reads that block's
// link word before its CAS. If another thread wins the CAS in between, it now
// owns that block and starts writing a payload into it -- over the very bytes
// the first thread is reading as a link. The stale read itself is harmless,
// because the CAS then fails and retries; what is not harmless is that the two
// threads are touching the same bytes with no synchronisation, one of them
// writing arbitrary user data. No amount of tagging fixes that, because the
// tag protects the head, not the block.
//
// Keeping the links outside the blocks removes the aliasing entirely: a link
// word is only ever touched by the allocator, never by a payload. It costs
// 8 bytes per block, which is the right trade for deleting a class of bug.
//
// This is deliberately NOT the MPMC ring protocol. That is phase 3, and it gets
// a formal model. This is the simpler allocation layer underneath it.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "lockstep/core/atomic_ops.hpp"

namespace ls {

// Blocks are cache-line aligned so two publishers writing adjacent blocks do
// not contend on the same line.
inline constexpr std::size_t arena_block_align = 64;
inline constexpr std::size_t max_size_classes = 16;

struct pool_desc {
  std::uint64_t block_size;
  std::uint64_t block_count;
  std::uint64_t storage_offset;  // segment-relative, start of block storage
  std::uint64_t links_offset;    // segment-relative, block_count uint64 links
  std::atomic<std::uint64_t> free_head;
  std::atomic<std::uint64_t> live_blocks;
};

struct arena_header {
  std::uint32_t class_count;
  std::uint32_t reserved;
  std::uint64_t total_bytes;
  pool_desc pools[max_size_classes];
};

static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "the arena free list must be lock-free to be shared across processes");

// One entry of the pool layout requested at construction time.
struct pool_spec {
  std::size_t block_size;
  std::size_t block_count;
};

namespace detail {

inline constexpr std::uint64_t arena_index_bits = 48;
inline constexpr std::uint64_t arena_index_mask = (1ull << arena_index_bits) - 1ull;

constexpr std::uint64_t arena_pack(std::uint64_t tag, std::uint64_t index) noexcept {
  return (tag << arena_index_bits) | (index & arena_index_mask);
}
constexpr std::uint64_t arena_index(std::uint64_t head) noexcept {
  return head & arena_index_mask;
}
constexpr std::uint64_t arena_tag(std::uint64_t head) noexcept {
  return head >> arena_index_bits;
}

constexpr std::size_t align_up(std::size_t v, std::size_t a) noexcept {
  return (v + a - 1) / a * a;
}

}  // namespace detail

// A view over an arena_header that already lives inside a mapped segment.
// Holds no state of its own beyond the two pointers, so each process builds its
// own arena object over the same shared header.
class arena {
 public:
  arena() = default;
  arena(void* segment_base, arena_header* hdr) noexcept
      : base_(static_cast<char*>(segment_base)), hdr_(hdr) {}

  // Bytes of segment needed for a given pool layout, header included.
  static std::size_t bytes_required(const pool_spec* specs, std::size_t count) noexcept {
    std::size_t total = detail::align_up(sizeof(arena_header), arena_block_align);
    for (std::size_t i = 0; i < count; ++i) {
      total += detail::align_up(specs[i].block_size, arena_block_align) *
               specs[i].block_count;
      // Free-list links, held outside the blocks; see the header comment.
      total += detail::align_up(specs[i].block_count * sizeof(std::uint64_t),
                                arena_block_align);
    }
    return total;
  }

  // Lay out the pools and thread every free list. Called once, by the process
  // that created the segment, before any other process attaches.
  static arena construct(void* segment_base, std::size_t arena_offset,
                         const pool_spec* specs, std::size_t count) noexcept {
    auto* b = static_cast<char*>(segment_base);
    auto* hdr = reinterpret_cast<arena_header*>(b + arena_offset);

    if (count > max_size_classes) count = max_size_classes;
    hdr->class_count = static_cast<std::uint32_t>(count);
    hdr->reserved = 0;

    std::size_t cursor =
        arena_offset + detail::align_up(sizeof(arena_header), arena_block_align);

    for (std::size_t i = 0; i < count; ++i) {
      const std::size_t bs = detail::align_up(specs[i].block_size, arena_block_align);
      pool_desc& p = hdr->pools[i];
      p.block_size = bs;
      p.block_count = specs[i].block_count;
      p.storage_offset = cursor;
      p.live_blocks.store(0, std::memory_order_relaxed);
      cursor += bs * specs[i].block_count;

      // The link array follows the blocks it describes.
      p.links_offset = cursor;
      auto* links = reinterpret_cast<std::uint64_t*>(b + cursor);

      // Thread the free list: block k points at block k+1, last points at 0.
      // Indices are stored 1-based so that 0 can mean "empty".
      for (std::size_t k = 0; k < specs[i].block_count; ++k)
        links[k] = (k + 1 < specs[i].block_count) ? (k + 2) : 0;

      p.free_head.store(detail::arena_pack(0, specs[i].block_count ? 1 : 0),
                        std::memory_order_relaxed);

      cursor += detail::align_up(specs[i].block_count * sizeof(std::uint64_t),
                                 arena_block_align);
    }

    hdr->total_bytes = cursor - arena_offset;
    std::atomic_thread_fence(std::memory_order_release);
    return arena(segment_base, hdr);
  }

  bool valid() const noexcept { return hdr_ != nullptr; }
  std::uint32_t class_count() const noexcept { return hdr_->class_count; }
  const pool_desc& pool(std::size_t i) const noexcept { return hdr_->pools[i]; }

  // Pop a block big enough for `bytes`. Returns a segment-relative offset, or 0
  // if every pool that could serve the request is exhausted. Never blocks,
  // never allocates, never throws.
  std::uint64_t allocate(std::size_t bytes) noexcept {
    for (std::uint32_t i = 0; i < hdr_->class_count; ++i) {
      pool_desc& p = hdr_->pools[i];
      if (p.block_size < bytes) continue;

      std::uint64_t head = p.free_head.load(std::memory_order_acquire);
      for (;;) {
        const std::uint64_t idx = detail::arena_index(head);
        if (idx == 0) break;  // this class is empty, try the next one up

        const std::uint64_t off = p.storage_offset + (idx - 1) * p.block_size;
        // Relaxed atomic: another thread may be recycling this entry right now.
        // The value may be stale, in which case the CAS below fails and we
        // retry -- but the ACCESS must not be a data race.
        auto* links = reinterpret_cast<std::uint64_t*>(base_ + p.links_offset);
        const std::uint64_t next = detail::relaxed_load(links[idx - 1]);
        const std::uint64_t updated =
            detail::arena_pack(detail::arena_tag(head) + 1, next);

        if (p.free_head.compare_exchange_weak(head, updated,
                                              std::memory_order_acq_rel,
                                              std::memory_order_acquire)) {
          p.live_blocks.fetch_add(1, std::memory_order_relaxed);
          return off;
        }
      }
    }
    return 0;
  }

  // Push a block back. The offset must be one this arena handed out.
  void deallocate(std::uint64_t offset) noexcept {
    if (offset == 0) return;
    for (std::uint32_t i = 0; i < hdr_->class_count; ++i) {
      pool_desc& p = hdr_->pools[i];
      const std::uint64_t lo = p.storage_offset;
      const std::uint64_t hi = lo + p.block_size * p.block_count;
      if (offset < lo || offset >= hi) continue;

      const std::uint64_t idx = (offset - lo) / p.block_size + 1;
      auto* links = reinterpret_cast<std::uint64_t*>(base_ + p.links_offset);

      std::uint64_t head = p.free_head.load(std::memory_order_acquire);
      for (;;) {
        detail::relaxed_store(links[idx - 1], detail::arena_index(head));
        const std::uint64_t updated =
            detail::arena_pack(detail::arena_tag(head) + 1, idx);
        if (p.free_head.compare_exchange_weak(head, updated,
                                              std::memory_order_acq_rel,
                                              std::memory_order_acquire)) {
          p.live_blocks.fetch_sub(1, std::memory_order_relaxed);
          return;
        }
      }
    }
  }

  // Blocks currently handed out, summed over every pool. Diagnostic only.
  std::uint64_t live_blocks() const noexcept {
    std::uint64_t n = 0;
    for (std::uint32_t i = 0; i < hdr_->class_count; ++i)
      n += hdr_->pools[i].live_blocks.load(std::memory_order_relaxed);
    return n;
  }

  std::uint64_t block_size_of(std::uint64_t offset) const noexcept {
    for (std::uint32_t i = 0; i < hdr_->class_count; ++i) {
      const pool_desc& p = hdr_->pools[i];
      if (offset >= p.storage_offset &&
          offset < p.storage_offset + p.block_size * p.block_count) {
        return p.block_size;
      }
    }
    return 0;
  }

  template <class T>
  T* at(std::uint64_t offset) const noexcept {
    return reinterpret_cast<T*>(base_ + offset);
  }

 private:
  char* base_ = nullptr;
  arena_header* hdr_ = nullptr;
};

}  // namespace ls
