// Minimal assertion harness. Each test is a plain executable that returns
// non-zero on failure; CTest is the actual test runner. Kept dependency-free so
// the tree configures and builds with nothing but CMake and a compiler.
#pragma once

#include <cstdio>

namespace ls::test {

inline int failures = 0;

inline void fail(const char* expr, const char* file, int line) {
  std::fprintf(stderr, "  FAIL %s:%d\n    %s\n", file, line, expr);
  ++failures;
}

// Routed through a function rather than an `if` in the macro so that checking a
// constexpr predicate does not trip -Wtautological-compare / MSVC C4127.
inline bool check(bool ok, const char* expr, const char* file, int line) {
  if (!ok) fail(expr, file, line);
  return ok;
}

inline int summary(const char* name) {
  if (failures != 0) {
    std::fprintf(stderr, "[%s] %d check(s) failed\n", name, failures);
    return 1;
  }
  std::printf("[%s] ok\n", name);
  return 0;
}

}  // namespace ls::test

#define LS_CHECK(expr) \
  ::ls::test::check(static_cast<bool>(expr), #expr, __FILE__, __LINE__)

#define LS_CHECK_EQ(a, b)                                             \
  ::ls::test::check(static_cast<bool>((a) == (b)), #a " == " #b, \
                    __FILE__, __LINE__)
