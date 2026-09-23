// core/process.hpp : is that process still alive?
//
// Two places need this and neither can include the other: the registry, to
// steal a topic-creation lock from a process that died holding it, and the
// reaper, to decide whether a publisher is gone. So it lives here, in one
// implementation, rather than as two that can drift apart.
//
// LINUX. pidfd_open (5.3+) refers to the process itself rather than to its
// number, so it cannot be fooled by pid recycling. Where it is missing the code
// falls back to kill(pid, 0). Both have the same two known limits, stated
// rather than papered over:
//
//   * A zombie -- dead but unreaped by its parent -- still answers "alive". For
//     this bus that is the right answer anyway: a zombie has certainly stopped
//     publishing, but its pid is not yet reusable, so nothing else can be
//     mistaken for it.
//   * kill(2) alone can be fooled by pid recycling. The window is enormous next
//     to a heartbeat interval, and a recycled pid will not be updating the
//     topic's heartbeat, so the reaper closes it in practice.
//
// WINDOWS. OpenProcess plus a zero-timeout wait. GetExitCodeProcess is the more
// obvious call and is deliberately NOT used: it reports STILL_ACTIVE (259) for
// a running process, which is indistinguishable from a process that exited with
// code 259. WaitForSingleObject has no such ambiguity -- WAIT_TIMEOUT means
// still running, WAIT_OBJECT_0 means signalled, i.e. exited.
#pragma once

#include <cstdint>
#include <string>

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
#include <csignal>
#include <unistd.h>
#if defined(__linux__)
#include <sys/syscall.h>
#endif
#endif

namespace ls {

inline std::uint32_t self_pid() noexcept {
#if defined(_WIN32)
  return static_cast<std::uint32_t>(::GetCurrentProcessId());
#else
  return static_cast<std::uint32_t>(::getpid());
#endif
}

inline bool process_alive(std::uint32_t pid) noexcept {
  if (pid == 0) return false;

#if defined(_WIN32)
  // PROCESS_QUERY_LIMITED_INFORMATION is the least privilege that still allows
  // a wait, so this works across integrity levels where a fuller right would be
  // refused.
  HANDLE h = ::OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION | SYNCHRONIZE, FALSE,
                           static_cast<DWORD>(pid));
  if (h == nullptr) {
    // Access denied means it exists but is not ours to inspect: alive.
    return ::GetLastError() == ERROR_ACCESS_DENIED;
  }
  const DWORD w = ::WaitForSingleObject(h, 0);
  ::CloseHandle(h);
  return w == WAIT_TIMEOUT;
#else
#if defined(__linux__) && defined(SYS_pidfd_open)
  const long fd = ::syscall(SYS_pidfd_open, static_cast<int>(pid), 0u);
  if (fd >= 0) {
    ::close(static_cast<int>(fd));
    return true;
  }
  if (errno == ESRCH) return false;
  if (errno == EPERM) return true;
  // ENOSYS on an older kernel: fall through to kill(2).
#endif
  return ::kill(static_cast<int>(pid), 0) == 0 || errno == EPERM;
#endif
}

// Sleep for roughly `us` microseconds.
//
// Windows has no usleep and Sleep() only resolves to milliseconds, so a
// sub-millisecond request is rounded UP to 1ms rather than down to zero: a
// caller asking for a short pause wants to yield, and a zero-length Sleep on
// Windows does not reliably do that. Callers using this for rate limiting get
// a coarser tick on Windows, which is fine -- the demo is not a real-time
// workload and says so.
inline void sleep_us(unsigned us) noexcept {
#if defined(_WIN32)
  ::Sleep(us == 0 ? 0 : (us < 1000 ? 1 : static_cast<DWORD>(us / 1000)));
#else
  ::usleep(us);
#endif
}

// A writable scratch path for a journal or a dump. /tmp does not exist on
// Windows, so honour TEMP/TMP there and fall back to the current directory.
inline std::string temp_path(const std::string& filename) {
#if defined(_WIN32)
  char buf[MAX_PATH + 1] = {};
  const DWORD n = ::GetTempPathA(MAX_PATH, buf);
  const std::string dir = (n > 0 && n <= MAX_PATH) ? std::string(buf, n) : std::string(".\\");
  return dir + filename;
#else
  return "/tmp/" + filename;
#endif
}

}  // namespace ls
