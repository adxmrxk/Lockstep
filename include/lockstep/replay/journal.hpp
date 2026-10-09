// replay/journal.hpp : the append-only record of everything a run depended on.
//
// A ROS 2 bag replays the MESSAGES. That is not enough to reproduce a bug,
// because a node's behaviour depends on more than its inputs: it depends on
// what the clock said, on which callback ran first, and on which messages it
// actually managed to see rather than lose to an overrun. Replay the messages
// alone and you get a different run that happens to share its inputs.
//
// So this journals every decision, not just the data:
//
//   record_kind::message   a message AS DELIVERED to a specific callback, with
//                          its bytes, in dispatch order
//   record_kind::clock     the value a clock read returned
//   record_kind::output    something the node produced, for hashing
//
// On replay, reads of all three are served from the journal instead of from the
// world. The callback cannot tell the difference, which is the entire point.
//
// The file is mmap'd and append-only. Zero-copy is what makes journalling every
// message affordable in the first place -- there is no serialisation step, the
// bytes are already in their final form, so recording is a memcpy of a message
// that was never copied on the transport path.
#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>

#include "lockstep/core/clock.hpp"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <cerrno>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace ls {

inline constexpr std::uint64_t journal_magic = 0x4c534a524e4c3032ull;  // "LSJRNL02"
inline constexpr std::uint64_t journal_magic_v1 = 0x4c534a524e4c3031ull;

enum class record_kind : std::uint32_t {
  message = 1,
  clock = 2,
  output = 3,
};

// How the record area is used.
//
//   linear  fill once, from the start, and stop when full. What you want when
//           recording a bounded run you intend to keep all of.
//
//   ring    keep the most recent window and throw the rest away, so a robot can
//           run for hours inside a fixed amount of memory. What you want when
//           the interesting thing has not happened yet.
enum class journal_mode : std::uint32_t { linear = 0, ring = 1 };

// Ring mode splits the record area in two and writes into one half at a time.
//
// The obvious design is a byte-level circular buffer, and it is worse. Records
// are variable length, so a wrapping write either straddles the end of the
// buffer (now a record is in two pieces and every reader has to know that) or
// needs skip markers padding out the tail, and evicting the oldest record to
// make room means walking forward from the head on the hot path.
//
// Halves avoid all of it. A record never straddles anything, each half stays an
// ordinary linear run of records, and rotating is two stores. The cost is that
// the retained window varies between one half and two, so "keeps the last 30
// seconds" is really "keeps the last 30 to 60". For a flight recorder that is a
// fine trade, and it is the same trade perf and most trace buffers make.
struct journal_header {
  std::uint64_t magic;
  std::uint64_t capacity;        // bytes in the record area
  std::uint64_t used;            // live bytes, across both halves in ring mode
  std::uint64_t records;         // live records
  std::uint64_t dropped;         // records discarded by rotation
  std::uint64_t half;            // bytes per half; 0 in linear mode
  std::uint64_t half_bytes[2];
  std::uint64_t half_records[2];
  std::uint32_t mode;
  std::uint32_t active;          // which half is being written
  std::uint64_t rotations;
};
static_assert(sizeof(journal_header) == 96, "journal header is a fixed layout");

// Fixed-size prefix; `bytes` of payload follow immediately, then padding to the
// next 8-byte boundary.
struct record_header {
  std::uint32_t kind;
  std::uint32_t callback;  // which callback this was delivered to
  std::uint64_t sequence;  // ring ticket, or a counter for clock/output
  std::uint64_t value;     // clock reading, or an output scalar
  std::uint32_t bytes;     // payload length
  std::uint32_t reserved;
};

struct record {
  record_kind kind = record_kind::message;
  std::uint32_t callback = 0;
  std::uint64_t sequence = 0;
  std::uint64_t value = 0;
  const std::uint8_t* payload = nullptr;
  std::uint32_t bytes = 0;
};

class journal {
 public:
  journal() = default;
  journal(const journal&) = delete;
  journal& operator=(const journal&) = delete;
  journal(journal&& o) noexcept { swap(o); }
  journal& operator=(journal&& o) noexcept {
    journal tmp(std::move(o));
    swap(tmp);
    return *this;
  }
  ~journal() { close(); }

  // Fill once and stop when full.
  static journal create(const std::string& path, std::size_t bytes) {
    return create_with_mode(path, bytes, journal_mode::linear);
  }

  // Keep the most recent window and discard the rest, so a long-running robot
  // records continuously inside a fixed budget. The window is between half and
  // all of `bytes`; see the note on journal_header.
  //
  // A ring journal is not meant to be replayed in place, though it can be.
  // snapshot() is the intended path: it writes the live window out as an
  // ordinary linear journal, which every existing reader handles unchanged.
  static journal create_ring(const std::string& path, std::size_t bytes) {
    return create_with_mode(path, bytes, journal_mode::ring);
  }

  static journal create_with_mode(const std::string& path, std::size_t bytes,
                                  journal_mode mode) {
    journal j;
    j.writable_ = true;
    j.size_ = bytes;
    j.path_ = path;

#if defined(_WIN32)
    HANDLE f = ::CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                             CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (f == INVALID_HANDLE_VALUE) throw_last_error("CreateFile", path);

    // The mapping size sets the file size, so no separate truncate is needed.
    const DWORD hi = static_cast<DWORD>((static_cast<std::uint64_t>(bytes) >> 32) & 0xffffffffu);
    const DWORD lo = static_cast<DWORD>(static_cast<std::uint64_t>(bytes) & 0xffffffffu);
    HANDLE m = ::CreateFileMappingA(f, nullptr, PAGE_READWRITE, hi, lo, nullptr);
    if (m == nullptr) {
      const DWORD e = ::GetLastError();
      ::CloseHandle(f);
      throw_win32("CreateFileMapping", path, e);
    }
    void* p = ::MapViewOfFile(m, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (p == nullptr) {
      const DWORD e = ::GetLastError();
      ::CloseHandle(m);
      ::CloseHandle(f);
      throw_win32("MapViewOfFile", path, e);
    }
    j.file_ = f;
    j.mapping_ = m;
    j.base_ = static_cast<std::uint8_t*>(p);
#else
    const int fd = ::open(path.c_str(), O_CREAT | O_TRUNC | O_RDWR, 0644);
    if (fd < 0) throw_errno("open", path);
    if (::ftruncate(fd, static_cast<off_t>(bytes)) != 0) {
      const int e = errno;
      ::close(fd);
      throw_errno("ftruncate", path, e);
    }
    void* p = ::mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
      const int e = errno;
      ::close(fd);
      throw_errno("mmap", path, e);
    }
    j.fd_ = fd;
    j.base_ = static_cast<std::uint8_t*>(p);
#endif

    auto* h = j.hdr();
    std::memset(static_cast<void*>(h), 0, sizeof(journal_header));
    h->magic = journal_magic;
    h->capacity = bytes - sizeof(journal_header);
    h->mode = static_cast<std::uint32_t>(mode);
    if (mode == journal_mode::ring) {
      // Both halves 8-aligned, so a record header is never misaligned.
      h->half = (h->capacity / 2) & ~7ull;
      if (h->half == 0) {
        throw std::runtime_error("lockstep: " + path +
                                 " is too small for a ring journal");
      }
    }
    return j;
  }

  static journal open_read(const std::string& path) {
    journal j;
    j.writable_ = false;
    j.path_ = path;

#if defined(_WIN32)
    HANDLE f = ::CreateFileA(path.c_str(), GENERIC_READ, FILE_SHARE_READ, nullptr,
                             OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (f == INVALID_HANDLE_VALUE) throw_last_error("CreateFile", path);

    ::LARGE_INTEGER li{};
    if (!::GetFileSizeEx(f, &li)) {
      const DWORD e = ::GetLastError();
      ::CloseHandle(f);
      throw_win32("GetFileSizeEx", path, e);
    }
    j.size_ = static_cast<std::size_t>(li.QuadPart);

    HANDLE m = ::CreateFileMappingA(f, nullptr, PAGE_READONLY, 0, 0, nullptr);
    if (m == nullptr) {
      const DWORD e = ::GetLastError();
      ::CloseHandle(f);
      throw_win32("CreateFileMapping", path, e);
    }
    void* p = ::MapViewOfFile(m, FILE_MAP_READ, 0, 0, 0);
    if (p == nullptr) {
      const DWORD e = ::GetLastError();
      ::CloseHandle(m);
      ::CloseHandle(f);
      throw_win32("MapViewOfFile", path, e);
    }
    j.file_ = f;
    j.mapping_ = m;
    j.base_ = static_cast<std::uint8_t*>(p);
#else
    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) throw_errno("open", path);
    struct ::stat st {};
    if (::fstat(fd, &st) != 0) {
      const int e = errno;
      ::close(fd);
      throw_errno("fstat", path, e);
    }
    j.size_ = static_cast<std::size_t>(st.st_size);
    void* p = ::mmap(nullptr, j.size_, PROT_READ, MAP_SHARED, fd, 0);
    if (p == MAP_FAILED) {
      const int e = errno;
      ::close(fd);
      throw_errno("mmap", path, e);
    }
    j.fd_ = fd;
    j.base_ = static_cast<std::uint8_t*>(p);
#endif

    if (j.hdr()->magic == journal_magic_v1) {
      // v1 had a 32 byte header and no ring fields, so its records start where
      // v2 keeps its bookkeeping. Reading one as v2 would hand back record
      // bytes as if they were offsets, which fails in confusing ways rather
      // than obvious ones.
      throw std::runtime_error("lockstep: " + path +
                               " is a v1 journal, recorded before ring mode."
                               " Re-record it.");
    }
    if (j.hdr()->magic != journal_magic)
      throw std::runtime_error("lockstep: " + path + " is not a lockstep journal");
    return j;
  }

  journal_mode mode() const noexcept {
    return static_cast<journal_mode>(hdr()->mode);
  }

  // Records thrown away by rotation. Non-zero means the window moved past the
  // start of the run, which matters: a snapshot is then a window, not the whole
  // thing, and its output hash will not match the full run's.
  std::uint64_t dropped() const noexcept { return hdr()->dropped; }
  std::uint64_t rotations() const noexcept { return hdr()->rotations; }

  bool valid() const noexcept { return base_ != nullptr; }
  std::uint64_t records() const noexcept { return hdr()->records; }
  std::uint64_t used() const noexcept { return hdr()->used; }

  // Append one record. Returns false if the journal is full rather than
  // throwing: a recording node must not unwind on its hot path.
  bool append(record_kind kind, std::uint32_t callback, std::uint64_t sequence,
              std::uint64_t value, const void* payload, std::uint32_t bytes) noexcept {
    if (!writable_) return false;
    auto* h = hdr();
    const std::uint64_t need = align8(sizeof(record_header) + bytes);

    std::uint64_t at = 0;
    if (h->half == 0) {
      // Linear: one run of records, stop when it is full.
      if (h->used + need > h->capacity) return false;
      at = h->used;
      h->half_bytes[0] = h->used + need;
      h->half_records[0] = h->records + 1;
    } else {
      // Ring: rotate when the active half cannot take the record. The half we
      // rotate INTO is the older of the two, so its contents are what gets
      // thrown away, and the half we just finished is kept whole.
      if (need > h->half) return false;  // never fits, in either half
      std::uint32_t a = h->active;
      if (h->half_bytes[a] + need > h->half) {
        const std::uint32_t other = a ^ 1u;
        h->dropped += h->half_records[other];
        h->used -= h->half_bytes[other];
        h->records -= h->half_records[other];
        h->half_bytes[other] = 0;
        h->half_records[other] = 0;
        h->active = other;
        ++h->rotations;
        a = other;
      }
      at = static_cast<std::uint64_t>(a) * h->half + h->half_bytes[a];
      h->half_bytes[a] += need;
      ++h->half_records[a];
    }

    auto* rh = reinterpret_cast<record_header*>(area() + at);
    rh->kind = static_cast<std::uint32_t>(kind);
    rh->callback = callback;
    rh->sequence = sequence;
    rh->value = value;
    rh->bytes = bytes;
    rh->reserved = 0;
    if (bytes > 0 && payload != nullptr)
      std::memcpy(area() + at + sizeof(record_header), payload, bytes);

    h->used += need;
    ++h->records;
    return true;
  }

  // Write the live window out as an ordinary LINEAR journal.
  //
  // This is how a ring journal becomes something you replay: the result has no
  // ring bookkeeping, holds the records in order, and is read by code that
  // knows nothing about rotation.
  //
  // NOT signal-safe. It opens a file and maps it, so a crash handler should set
  // a flag and let the main loop call this.
  bool snapshot(const std::string& path) const {
    auto* h = hdr();
    try {
      journal out = create(path, sizeof(journal_header) + h->used + 64);
      record r;
      const std::uint64_t saved = cursor_;
      cursor_ = 0;
      while (next(r)) {
        if (!out.append(r.kind, r.callback, r.sequence, r.value, r.payload,
                        r.bytes)) {
          cursor_ = saved;
          return false;
        }
      }
      cursor_ = saved;
      return true;
    } catch (const std::exception&) {
      return false;
    }
  }

  // Sequential read, oldest record first. Returns false at the end.
  //
  // `cursor_` counts bytes through the LIVE window rather than through the
  // file, so that ring mode reads in age order without every caller having to
  // know which half is older. physical() does the mapping.
  bool next(record& out) const noexcept {
    auto* h = hdr();
    if (cursor_ >= h->used) return false;
    const std::uint64_t at = physical(cursor_);
    const auto* rh = reinterpret_cast<const record_header*>(area() + at);
    out.kind = static_cast<record_kind>(rh->kind);
    out.callback = rh->callback;
    out.sequence = rh->sequence;
    out.value = rh->value;
    out.bytes = rh->bytes;
    out.payload = rh->bytes ? (area() + at + sizeof(record_header)) : nullptr;
    cursor_ += align8(sizeof(record_header) + rh->bytes);
    return true;
  }

  void rewind() const noexcept { cursor_ = 0; }

  // FNV-1a over every record's header fields and payload. Two journals with
  // this hash equal recorded the same run.
  std::uint64_t content_hash() const noexcept {
    std::uint64_t hv = 14695981039346656037ull;
    const auto mix = [&hv](const void* p, std::size_t n) {
      const auto* b = static_cast<const std::uint8_t*>(p);
      for (std::size_t i = 0; i < n; ++i) {
        hv ^= b[i];
        hv *= 1099511628211ull;
      }
    };
    std::uint64_t c = 0;
    auto* h = hdr();
    while (c < h->used) {
      const std::uint64_t at = physical(c);
      const auto* rh = reinterpret_cast<const record_header*>(area() + at);
      mix(&rh->kind, sizeof(rh->kind));
      mix(&rh->callback, sizeof(rh->callback));
      mix(&rh->sequence, sizeof(rh->sequence));
      mix(&rh->value, sizeof(rh->value));
      mix(&rh->bytes, sizeof(rh->bytes));
      if (rh->bytes) mix(area() + at + sizeof(record_header), rh->bytes);
      c += align8(sizeof(record_header) + rh->bytes);
    }
    return hv;
  }

  void close() noexcept {
#if defined(_WIN32)
    if (base_ != nullptr) ::UnmapViewOfFile(base_);
    if (mapping_ != nullptr) ::CloseHandle(mapping_);
    if (file_ != INVALID_HANDLE_VALUE && file_ != nullptr) ::CloseHandle(file_);
    mapping_ = nullptr;
    file_ = INVALID_HANDLE_VALUE;
#else
    if (base_ != nullptr) ::munmap(base_, size_);
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
#endif
    base_ = nullptr;
    size_ = 0;
  }

 private:
  static constexpr std::uint64_t align8(std::uint64_t v) noexcept {
    return (v + 7u) & ~7ull;
  }
  journal_header* hdr() const noexcept {
    return reinterpret_cast<journal_header*>(base_);
  }
  std::uint8_t* area() const noexcept { return base_ + sizeof(journal_header); }

  // Logical byte offset in the live window, to a byte offset in the record
  // area.
  //
  // Linear mode is the identity. Ring mode reads the older half first, which
  // is the one that is NOT active once anything has rotated. Before the first
  // rotation only half 0 has ever been written, so there is nothing to put in
  // front of it.
  std::uint64_t physical(std::uint64_t logical) const noexcept {
    auto* h = hdr();
    if (h->half == 0) return logical;
    const std::uint32_t first =
        h->rotations > 0 ? (h->active ^ 1u) : h->active;
    const std::uint64_t first_bytes = h->half_bytes[first];
    if (logical < first_bytes) {
      return static_cast<std::uint64_t>(first) * h->half + logical;
    }
    const std::uint32_t second = first ^ 1u;
    return static_cast<std::uint64_t>(second) * h->half +
           (logical - first_bytes);
  }

  void swap(journal& o) noexcept {
    std::swap(base_, o.base_);
    std::swap(size_, o.size_);
#if defined(_WIN32)
    std::swap(file_, o.file_);
    std::swap(mapping_, o.mapping_);
#else
    std::swap(fd_, o.fd_);
#endif
    std::swap(writable_, o.writable_);
    std::swap(cursor_, o.cursor_);
    path_.swap(o.path_);
  }

#if defined(_WIN32)
  [[noreturn]] static void throw_win32(const char* what, const std::string& p, DWORD e) {
    throw std::system_error(static_cast<int>(e), std::system_category(),
                            std::string(what) + " failed for journal " + p);
  }
  [[noreturn]] static void throw_last_error(const char* what, const std::string& p) {
    throw_win32(what, p, ::GetLastError());
  }
#else
  [[noreturn]] static void throw_errno(const char* what, const std::string& p,
                                       int e = errno) {
    throw std::system_error(e, std::generic_category(),
                            std::string(what) + " failed for journal " + p);
  }
#endif

  std::uint8_t* base_ = nullptr;
  std::size_t size_ = 0;
#if defined(_WIN32)
  HANDLE file_ = INVALID_HANDLE_VALUE;
  HANDLE mapping_ = nullptr;
#else
  int fd_ = -1;
#endif
  bool writable_ = false;
  mutable std::uint64_t cursor_ = 0;
  std::string path_;
};

// ---------------------------------------------------------------------------
// The clock a node must use if it wants to be replayable.
//
// A wall-clock read is a hidden input. Left alone it is the single most common
// reason a "replayed" run diverges, because the node branches on a value the
// replay never reproduced. Recording turns it into an ordinary journalled
// input; replay serves it back.
// ---------------------------------------------------------------------------
enum class run_mode : std::uint32_t { live = 0, record, replay };

class deterministic_clock {
 public:
  deterministic_clock() = default;
  deterministic_clock(run_mode m, journal* j) noexcept : mode_(m), journal_(j) {}

  std::uint64_t now_ns() noexcept {
    switch (mode_) {
      case run_mode::record: {
        const std::uint64_t t = ls::now_ns();
        journal_->append(record_kind::clock, 0, reads_++, t, nullptr, 0);
        return t;
      }
      case run_mode::replay: {
        // Served from the journal, so the node sees the same instant it saw
        // during the recorded run.
        record r;
        while (journal_->next(r)) {
          if (r.kind == record_kind::clock) {
            ++reads_;
            return r.value;
          }
        }
        return last_;
      }
      case run_mode::live:
      default:
        return ls::now_ns();
    }
  }

  run_mode mode() const noexcept { return mode_; }
  std::uint64_t reads() const noexcept { return reads_; }

 private:
  run_mode mode_ = run_mode::live;
  journal* journal_ = nullptr;
  std::uint64_t reads_ = 0;
  std::uint64_t last_ = 0;
};

// Running FNV-1a over whatever a node produces. Two runs that agree on this
// agree bit for bit on their output.
class output_hasher {
 public:
  void mix(const void* p, std::size_t n) noexcept {
    const auto* b = static_cast<const std::uint8_t*>(p);
    for (std::size_t i = 0; i < n; ++i) {
      h_ ^= b[i];
      h_ *= 1099511628211ull;
    }
    ++items_;
  }
  template <class T>
  void mix_value(const T& v) noexcept {
    mix(&v, sizeof(T));
  }
  std::uint64_t value() const noexcept { return h_; }
  std::uint64_t items() const noexcept { return items_; }
  void reset() noexcept {
    h_ = 14695981039346656037ull;
    items_ = 0;
  }

 private:
  std::uint64_t h_ = 14695981039346656037ull;
  std::uint64_t items_ = 0;
};

}  // namespace ls
