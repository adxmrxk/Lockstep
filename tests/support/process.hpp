// Spawning, killing and reaping a child process, on both platforms.
//
// The multi-process tests are the ones that actually prove anything -- a second
// process with its own address space and its own mapping base is the whole
// point of the bus -- so they cannot be quietly dropped on Windows just because
// fork() is not there. This is the thin shim that keeps them running on both.
//
// The kill is deliberately the harshest one each platform offers: SIGKILL and
// TerminateProcess. Neither can be caught, neither unwinds, neither runs an
// atexit handler. That is the failure mode phase 5 has to survive, and a
// gentler signal would be testing something easier than the real thing.
//
// Note the one honest asymmetry: SIGKILL and TerminateProcess stop the process
// at an arbitrary instruction, but Windows will not interrupt a thread inside a
// kernel call the way a signal can. In practice both land inside the ring's
// claim-to-commit window often enough for the crash tests to exercise it --
// test_crash_consistency asserts a non-zero heal count rather than assuming it.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lockstep/core/process.hpp"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace ls::test {

#if defined(_WIN32)
struct child_process {
  HANDLE handle = nullptr;
  std::uint32_t pid = 0;
  bool valid() const noexcept { return handle != nullptr; }
};
#else
struct child_process {
  pid_t handle = -1;
  std::uint32_t pid = 0;
  bool valid() const noexcept { return handle > 0; }
};
#endif

// Launch `exe` with `args`. Returns an invalid child_process on failure.
inline child_process spawn(const std::string& exe, const std::vector<std::string>& args) {
  child_process c;

#if defined(_WIN32)
  // CreateProcess wants one mutable command line, not an argv array. Quote each
  // element so paths containing spaces survive -- which on Windows they usually
  // do, since the build tree lives under a user profile.
  std::string cmd = "\"" + exe + "\"";
  for (const auto& a : args) cmd += " \"" + a + "\"";

  ::STARTUPINFOA si{};
  si.cb = sizeof(si);
  ::PROCESS_INFORMATION pi{};

  std::vector<char> mutable_cmd(cmd.begin(), cmd.end());
  mutable_cmd.push_back('\0');

  if (!::CreateProcessA(exe.c_str(), mutable_cmd.data(), nullptr, nullptr, FALSE, 0,
                        nullptr, nullptr, &si, &pi)) {
    return c;
  }
  ::CloseHandle(pi.hThread);  // we never use the thread handle
  c.handle = pi.hProcess;
  c.pid = static_cast<std::uint32_t>(pi.dwProcessId);
#else
  std::vector<char*> argv;
  std::string exe_copy = exe;
  std::vector<std::string> arg_copies = args;
  argv.push_back(exe_copy.data());
  for (auto& a : arg_copies) argv.push_back(a.data());
  argv.push_back(nullptr);

  const pid_t pid = ::fork();
  if (pid < 0) return c;
  if (pid == 0) {
    ::execv(exe.c_str(), argv.data());
    ::_exit(127);  // only reached if exec failed
  }
  c.handle = pid;
  c.pid = static_cast<std::uint32_t>(pid);
#endif
  return c;
}

// Kill without giving the child any chance to clean up.
inline void kill_hard(child_process& c) noexcept {
  if (!c.valid()) return;
#if defined(_WIN32)
  ::TerminateProcess(c.handle, 9);
#else
  ::kill(c.handle, SIGKILL);
#endif
}

// Wait for the child to finish and release its handle. Returns its exit code,
// or -1 if it could not be reaped. A killed child reports a non-zero code on
// both platforms; the tests only care that it stopped.
inline int wait_for(child_process& c) noexcept {
  if (!c.valid()) return -1;
#if defined(_WIN32)
  ::WaitForSingleObject(c.handle, INFINITE);
  DWORD code = 0;
  const BOOL ok = ::GetExitCodeProcess(c.handle, &code);
  ::CloseHandle(c.handle);
  c.handle = nullptr;
  return ok ? static_cast<int>(code) : -1;
#else
  int status = 0;
  if (::waitpid(c.handle, &status, 0) != c.handle) return -1;
  c.handle = -1;
  if (WIFEXITED(status)) return WEXITSTATUS(status);
  if (WIFSIGNALED(status)) return 128 + WTERMSIG(status);
  return -1;
#endif
}

// ls::sleep_us and ls::temp_path live in lockstep/core/process.hpp; they are
// useful to nodes as well as to tests, so they are not duplicated here.
using ls::sleep_us;

}  // namespace ls::test
