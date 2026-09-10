// shm/bus.hpp : the segment header, and the object that opens a whole bus.
//
// The segment has to describe itself. A subscriber is handed nothing but a
// name, maps the object, and must locate the registry and the arena from the
// bytes alone -- it cannot be passed pointers, because the mapping sits at a
// different address in its address space. So offset 0 holds a fixed header
// giving the offset of everything else.
//
//   bus b = bus::create("demo");     // publisher: lays out the segment
//   bus b = bus::open("demo");       // subscriber: reads the header and attaches
//
// The `ready` word is the initialisation barrier. create() lays out the
// registry and arena and only then stores ready, with release ordering; open()
// spins on it with acquire ordering. Without it a subscriber that wins the race
// to shm_open would read a zero-filled registry and conclude the bus is empty.
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

#include "lockstep/shm/arena.hpp"
#include "lockstep/shm/registry.hpp"
#include "lockstep/shm/segment.hpp"

namespace ls {

inline constexpr std::uint64_t bus_magic = 0x4c4f434b53544550ull;  // "LOCKSTEP"
inline constexpr std::uint32_t bus_version = 1;

struct segment_header {
  std::uint64_t magic;
  std::uint32_t version;
  std::uint32_t header_size;
  std::uint64_t segment_size;
  std::uint64_t registry_offset;
  std::uint64_t arena_offset;
  std::atomic<std::uint32_t> ready;
  std::uint32_t reserved;
};

// The pool layout a bus is created with. Defaults span the range the README
// describes -- small control messages up through a 4K camera frame -- at counts
// that keep a demo segment a few tens of megabytes rather than gigabytes.
inline std::vector<pool_spec> default_pools() {
  return {
      {64, 1024}, {256, 512}, {1024, 256}, {4096, 128},
      {64 * 1024, 32}, {1024 * 1024, 8}, {8 * 1024 * 1024, 2},
  };
}

class bus {
 public:
  bus() = default;

  static bus create(std::string_view name,
                    std::vector<pool_spec> pools = default_pools()) {
    const std::size_t hdr_bytes = align64(sizeof(segment_header));
    const std::size_t reg_bytes = align64(registry::bytes_required());
    const std::size_t arena_bytes = arena::bytes_required(pools.data(), pools.size());
    const std::size_t total = hdr_bytes + reg_bytes + arena_bytes;

    bus b;
    b.seg_ = segment::create(name, total);

    auto* h = b.seg_.at<segment_header>(0);
    h->magic = bus_magic;
    h->version = bus_version;
    h->header_size = static_cast<std::uint32_t>(hdr_bytes);
    h->segment_size = total;
    h->registry_offset = hdr_bytes;
    h->arena_offset = hdr_bytes + reg_bytes;
    h->reserved = 0;
    h->ready.store(0, std::memory_order_relaxed);

    b.reg_ = registry::construct(b.seg_.base(), h->registry_offset);
    b.arena_ = arena::construct(b.seg_.base(), h->arena_offset, pools.data(),
                                pools.size());

    // Everything above must be visible before anyone acts on ready.
    h->ready.store(1, std::memory_order_release);
    b.hdr_ = h;
    return b;
  }

  // Attach to a bus somebody else created. Waits up to `timeout` for the
  // creator to finish laying the segment out.
  static bus open(std::string_view name,
                  std::chrono::milliseconds timeout = std::chrono::milliseconds(2000)) {
    bus b;
    b.seg_ = segment::open(name);
    auto* h = b.seg_.at<segment_header>(0);

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (h->ready.load(std::memory_order_acquire) != 1) {
      if (std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("lockstep: timed out waiting for segment to initialise");
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    if (h->magic != bus_magic)
      throw std::runtime_error("lockstep: segment magic mismatch, not a lockstep bus");
    if (h->version != bus_version)
      throw std::runtime_error("lockstep: segment version mismatch");

    b.hdr_ = h;
    b.reg_ = registry(b.seg_.at<registry_header>(h->registry_offset));
    b.arena_ = arena(b.seg_.base(), b.seg_.at<arena_header>(h->arena_offset));
    return b;
  }

  bool valid() const noexcept { return hdr_ != nullptr; }
  segment& seg() noexcept { return seg_; }
  registry& topics() noexcept { return reg_; }
  arena& allocator() noexcept { return arena_; }
  const segment_header& header() const noexcept { return *hdr_; }

 private:
  static constexpr std::size_t align64(std::size_t v) noexcept {
    return (v + 63) / 64 * 64;
  }

  segment seg_;
  segment_header* hdr_ = nullptr;
  registry reg_;
  arena arena_;
};

}  // namespace ls
