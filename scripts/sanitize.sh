#!/usr/bin/env bash
# Run the suite under ThreadSanitizer, then AddressSanitizer + UBSan.
#
# These are not optional extras. TSan found two real bugs the TLA+ model could
# not, because the model is sequentially consistent and says nothing about
# barriers; ASan found a third that had been passing for the wrong reason.
# Run this before believing a green ctest.
set -uo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src="$here/.."
fail=0

run_with() {  # run_with <label> <flags>
  local label="$1" flags="$2"
  local build; build="$(mktemp -d)"
  echo "== $label =="
  cmake -S "$src" -B "$build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_CXX_FLAGS="$flags -g -O1 -fno-omit-frame-pointer" \
        -DCMAKE_EXE_LINKER_FLAGS="$flags" >/dev/null 2>&1 || {
          echo "   configure failed"; fail=1; return; }
  cmake --build "$build" -j >/dev/null 2>&1 || { echo "   build failed"; fail=1; return; }

  if ( cd "$build" && ctest --output-on-failure >/tmp/san.$$ 2>&1 ); then
    echo "   $(grep -E 'tests passed' /tmp/san.$$)"
  else
    echo "   FAILURES:"
    grep -E '\*\*\*Failed|WARNING: ThreadSanitizer|ERROR: AddressSanitizer|runtime error' \
      /tmp/san.$$ | sort -u | head -20
    fail=1
  fi
  rm -rf "$build" /tmp/san.$$
}

run_with "ThreadSanitizer"            "-fsanitize=thread"
run_with "AddressSanitizer + UBSan"   "-fsanitize=address,undefined"

if [ $fail -eq 0 ]; then echo; echo "all sanitizers clean"; else echo; echo "SANITIZER FAILURES"; fi
exit $fail
