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
// Ownership: create() unlinks the name when the owning segment is destroyed.
// open() never unlinks. A subscriber outliving its publisher keeps a valid
// mapping -- POSIX keeps the object alive until the last munmap -- which is
// what makes the crash-consistency work in phase 5 possible at all.
//
// Setup-time failures throw std::system_error. That is deliberate and is
// confined to setup: nothing on the publish or subscribe path throws, because
// the bus must not unwind on the hot path.
#pragma once

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#if defined(_WIN32)
#error "shm/segment.hpp is POSIX-only for now. The Win32 CreateFileMapping backend is not written yet."
#endif

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

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
    return s;
  }

  // Attach to an existing segment. The size comes from the object itself, so a
  // subscriber does not have to be told how big the bus is.
  static segment open(std::string_view name) {
    segment s;
    s.name_ = canonical(name);
    s.owner_ = false;

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
    return s;
  }

  // Remove a name without mapping it. Used to clear a segment left behind by a
  // process that died before it could unlink its own.
  static void unlink(std::string_view name) noexcept {
    ::shm_unlink(canonical(name).c_str());
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
    void* p = ::mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (p == MAP_FAILED) {
      const int err = errno;
      ::close(fd_);
      fd_ = -1;
      if (owner_) ::shm_unlink(name_.c_str());
      throw_errno("mmap", name_, err);
    }
    base_ = p;
  }

  void reset() noexcept {
    if (base_ != nullptr) ::munmap(base_, size_);
    if (fd_ >= 0) ::close(fd_);
    if (owner_ && !name_.empty()) ::shm_unlink(name_.c_str());
    base_ = nullptr;
    size_ = 0;
    fd_ = -1;
    owner_ = false;
    name_.clear();
  }

  void swap(segment& o) noexcept {
    std::swap(base_, o.base_);
    std::swap(size_, o.size_);
    std::swap(fd_, o.fd_);
    std::swap(owner_, o.owner_);
    name_.swap(o.name_);
  }

  // POSIX wants a leading slash and no others.
  static std::string canonical(std::string_view name) {
    std::string s;
    s.reserve(name.size() + 1);
    s.push_back('/');
    for (const char c : name) s.push_back(c == '/' ? '_' : c);
    return s;
  }

  [[noreturn]] static void throw_errno(const char* what, const std::string& name,
                                       int err = errno) {
    throw std::system_error(err, std::generic_category(),
                            std::string(what) + " failed for shm segment " + name);
  }

  void* base_ = nullptr;
  std::size_t size_ = 0;
  int fd_ = -1;
  bool owner_ = false;
  std::string name_;
};

}  // namespace ls
