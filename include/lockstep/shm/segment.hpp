// shm/segment.hpp : a named shared-memory segment, mapped into this process.
//
// This is the substrate everything above it stands on. One segment holds the
// registry and the arena; every process on the robot maps the same segment and
// sees the same bytes at a DIFFERENT base address, which is the entire reason
// phase 1's offset_ptr and relocatable-type machinery exists.
//
//   segment s = segment::create("lockstep-demo", 8u << 20);   // publisher
//   segment s = segment::open("lockstep-demo");               // subscriber
//
// Setup-time failures throw std::system_error. That is deliberate and is
// confined to setup: nothing on the publish or subscribe path throws, because
// the bus must not unwind on the hot path.
//
// ---------------------------------------------------------------------------
// TWO BACKENDS, AND WHERE THEY DIFFER
// ---------------------------------------------------------------------------
// POSIX: shm_open + ftruncate + mmap. The name lives in the filesystem
// namespace (/dev/shm), so it OUTLIVES every process that used it. create()
// therefore unlinks on destruction, and a process that dies without unlinking
// leaves a stale segment behind -- which is why unlink() is exposed and why the
// tests call it defensively before creating.
//
// Win32: CreateFileMapping against the page file. A section object is
// reference-counted by the kernel and disappears when the last handle closes,
// so there is nothing to unlink and nothing stale to clean up. unlink() is a
// documented no-op there rather than an error.
//
// The shared behaviour both backends guarantee, and which everything above
// depends on:
//   * create() refuses to attach to an existing name (exclusive creation);
//   * open() attaches to an existing one and fails if it is absent;
//   * a mapping stays valid for its holder even after the creator goes away.
//
// The last point is what makes phase 5's crash consistency possible at all, and
// both platforms give it for free: POSIX keeps an unlinked object alive until
// the last munmap, Win32 keeps the section alive until the last handle closes.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

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

class segment {
 public:
  segment() = default;

  segment(const segment&) = delete;
  segment& operator=(const segment&) = delete;

  segment(segment&& other) noexcept { swap(other); }
  segment& operator=(segment&& other) noexcept {
    segment tmp(std::move(other));
    swap(tmp);
    return *this;
  }

  ~segment() { reset(); }

  // Create a new segment. Fails if the name already exists, so two publishers
  // racing to create the same bus is an error rather than a silent takeover.
  static segment create(std::string_view name, std::size_t bytes) {
    segment s;
    s.name_ = canonical(name);
    s.owner_ = true;
    s.size_ = bytes;

#if defined(_WIN32)
    // INVALID_HANDLE_VALUE means "back this with the page file" rather than a
    // real file on disk, which is the Win32 equivalent of a POSIX shm object.
    const DWORD hi = static_cast<DWORD>((static_cast<std::uint64_t>(bytes) >> 32) & 0xffffffffu);
    const DWORD lo = static_cast<DWORD>(static_cast<std::uint64_t>(bytes) & 0xffffffffu);
    HANDLE h = ::CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, hi, lo,
                                    s.name_.c_str());
    if (h == nullptr) throw_last_error("CreateFileMapping", s.name_);
    if (::GetLastError() == ERROR_ALREADY_EXISTS) {
      // CreateFileMapping opens an existing section rather than failing, so
      // exclusivity has to be enforced here to match shm_open(O_EXCL).
      ::CloseHandle(h);
      throw std::system_error(static_cast<int>(ERROR_ALREADY_EXISTS),
                              std::system_category(),
                              "shm segment " + s.name_ + " already exists");
    }
    s.handle_ = h;
    s.map();
#else
    const int fd = ::shm_open(s.name_.c_str(), O_CREAT | O_EXCL | O_RDWR, 0600);
    if (fd < 0) throw_errno("shm_open(O_CREAT|O_EXCL)", s.name_);

    if (::ftruncate(fd, static_cast<off_t>(bytes)) != 0) {
      const int err = errno;
      ::close(fd);
      ::shm_unlink(s.name_.c_str());
      throw_errno("ftruncate", s.name_, err);
    }

    s.fd_ = fd;
    s.map();
#endif
    return s;
  }

  // Attach to an existing segment. The size comes from the object itself, so a
  // subscriber does not have to be told how big the bus is.
  static segment open(std::string_view name) {
    segment s;
    s.name_ = canonical(name);
    s.owner_ = false;

#if defined(_WIN32)
    HANDLE h = ::OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, s.name_.c_str());
    if (h == nullptr) throw_last_error("OpenFileMapping", s.name_);
    s.handle_ = h;
    s.map();  // size is recovered from the mapping itself; see map()
#else
    const int fd = ::shm_open(s.name_.c_str(), O_RDWR, 0600);
    if (fd < 0) throw_errno("shm_open", s.name_);

    struct ::stat st {};
    if (::fstat(fd, &st) != 0) {
      const int err = errno;
      ::close(fd);
      throw_errno("fstat", s.name_, err);
    }

    s.fd_ = fd;
    s.size_ = static_cast<std::size_t>(st.st_size);
    s.map();
#endif
    return s;
  }

  // Remove a name without mapping it. On POSIX this clears a segment left
  // behind by a process that died before it could unlink its own. On Win32 a
  // section has no filesystem name to remove and vanishes when the last handle
  // closes, so this is a no-op -- deliberately, rather than an error, so callers
  // can clean up defensively on both platforms with the same code.
  static void unlink([[maybe_unused]] std::string_view name) noexcept {
#if !defined(_WIN32)
    ::shm_unlink(canonical(name).c_str());
#endif
  }

  bool valid() const noexcept { return base_ != nullptr; }
  void* base() const noexcept { return base_; }
  std::size_t size() const noexcept { return size_; }
  const std::string& name() const noexcept { return name_; }
  bool owner() const noexcept { return owner_; }

  // Offset <-> address conversion. Everything stored inside the segment is
  // addressed by offset, because base_ differs in every process.
  template <class T>
  T* at(std::size_t offset) const noexcept {
    return reinterpret_cast<T*>(static_cast<char*>(base_) + offset);
  }

  std::size_t offset_of(const void* p) const noexcept {
    return static_cast<std::size_t>(static_cast<const char*>(p) -
                                    static_cast<const char*>(base_));
  }

  bool contains(const void* p, std::size_t bytes = 0) const noexcept {
    const auto* c = static_cast<const char*>(p);
    const auto* lo = static_cast<const char*>(base_);
    return c >= lo && (c + bytes) <= (lo + size_);
  }

 private:
  void map() {
#if defined(_WIN32)
    void* p = ::MapViewOfFile(handle_, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (p == nullptr) {
      const DWORD err = ::GetLastError();
      ::CloseHandle(handle_);
      handle_ = nullptr;
      throw std::system_error(static_cast<int>(err), std::system_category(),
                              "MapViewOfFile failed for shm segment " + name_);
    }
    base_ = p;

    if (size_ == 0) {
      // open() does not know the section size up front -- Win32 has no fstat
      // for a section object -- so recover it from the mapping. RegionSize is
      // the section size rounded up to the page size, so size() can read very
      // slightly high on Windows. Nothing depends on it being exact: the bus
      // stores its own authoritative length in segment_header::segment_size,
      // and contains() only uses this as an upper bound.
      ::MEMORY_BASIC_INFORMATION mbi{};
      if (::VirtualQuery(p, &mbi, sizeof(mbi)) == 0) {
        const DWORD err = ::GetLastError();
        ::UnmapViewOfFile(p);
        ::CloseHandle(handle_);
        base_ = nullptr;
        handle_ = nullptr;
        throw std::system_error(static_cast<int>(err), std::system_category(),
                                "VirtualQuery failed for shm segment " + name_);
      }
      size_ = static_cast<std::size_t>(mbi.RegionSize);
    }
#else
    void* p = ::mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (p == MAP_FAILED) {
      const int err = errno;
      ::close(fd_);
      fd_ = -1;
      if (owner_) ::shm_unlink(name_.c_str());
      throw_errno("mmap", name_, err);
    }
    base_ = p;
#endif
  }

  void reset() noexcept {
#if defined(_WIN32)
    if (base_ != nullptr) ::UnmapViewOfFile(base_);
    if (handle_ != nullptr) ::CloseHandle(handle_);
    handle_ = nullptr;
#else
    if (base_ != nullptr) ::munmap(base_, size_);
    if (fd_ >= 0) ::close(fd_);
    if (owner_ && !name_.empty()) ::shm_unlink(name_.c_str());
    fd_ = -1;
#endif
    base_ = nullptr;
    size_ = 0;
    owner_ = false;
    name_.clear();
  }

  void swap(segment& o) noexcept {
    std::swap(base_, o.base_);
    std::swap(size_, o.size_);
    std::swap(owner_, o.owner_);
#if defined(_WIN32)
    std::swap(handle_, o.handle_);
#else
    std::swap(fd_, o.fd_);
#endif
    name_.swap(o.name_);
  }

  // POSIX wants a leading slash and no others. Win32 wants a namespace prefix;
  // "Local\" keeps the segment inside the caller's logon session, which is the
  // closest match to a /dev/shm object's reach and avoids needing the
  // SeCreateGlobalPrivilege that "Global\" would.
  static std::string canonical(std::string_view name) {
    std::string s;
#if defined(_WIN32)
    s.reserve(name.size() + 6);
    s += "Local\\";
    for (const char c : name) s.push_back((c == '/' || c == '\\') ? '_' : c);
#else
    s.reserve(name.size() + 1);
    s.push_back('/');
    for (const char c : name) s.push_back(c == '/' ? '_' : c);
#endif
    return s;
  }

#if defined(_WIN32)
  [[noreturn]] static void throw_last_error(const char* what, const std::string& name) {
    throw std::system_error(static_cast<int>(::GetLastError()), std::system_category(),
                            std::string(what) + " failed for shm segment " + name);
  }
#else
  [[noreturn]] static void throw_errno(const char* what, const std::string& name,
                                       int err = errno) {
    throw std::system_error(err, std::generic_category(),
                            std::string(what) + " failed for shm segment " + name);
  }
#endif

  void* base_ = nullptr;
  std::size_t size_ = 0;
  bool owner_ = false;
  std::string name_;
#if defined(_WIN32)
  HANDLE handle_ = nullptr;
#else
  int fd_ = -1;
#endif
};

}  // namespace ls
