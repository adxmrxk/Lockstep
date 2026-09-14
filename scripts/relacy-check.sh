#!/usr/bin/env bash
#
# Check the ring's MEMORY ORDERING with Relacy.
#
# This is the half TLA+ cannot do. docs/tla/Ring.tla verifies the algorithm, but
# every action in it is sequentially consistent, so a barrier that is too weak
# is invisible there. Relacy simulates the C++11 memory model and explores the
# reorderings acquire/release actually permit.
#
# Two builds, and BOTH outcomes are asserted:
#   default                          release/acquire -> must PASS
#   -DLOCKSTEP_RELACY_RELAXED_STATE  relaxed state   -> must FAIL
#
# The second is not decoration. Without it, a passing run cannot be told apart
# from a model too weak to detect anything -- the same reason the replay test
# carries a live-rerun control.
#
# Relacy is header-only and needs no root:
#   git clone --depth 1 https://github.com/dvyukov/relacy.git ~/relacy
set -uo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
src="$here/../verification/relacy/ring_seqlock.cpp"
relacy="${RELACY_DIR:-$HOME/relacy}"
out="${TMPDIR:-/tmp}"

if [ ! -f "$relacy/relacy/relacy_std.hpp" ]; then
  echo "[relacy] fetching relacy into $relacy"
  git clone --depth 1 https://github.com/dvyukov/relacy.git "$relacy" || {
    echo "[relacy] clone failed; set RELACY_DIR to an existing checkout" >&2
    exit 1; }
fi

CXX="${CXX:-g++}"
# Relacy predates C++17 and does not build as C++20; it does not need to, since
# it is modelling the memory model rather than compiling the library.
STD=-std=c++14

fail=0

echo "== release/acquire (the implementation): expected to PASS =="
if ! "$CXX" $STD -I "$relacy" -o "$out/ls_relacy_ok" "$src" 2>"$out/ls_relacy_ok.log"; then
  echo "   FAIL: did not compile"; sed -n '1,15p' "$out/ls_relacy_ok.log"; fail=1
elif "$out/ls_relacy_ok" >"$out/ls_relacy_ok.run" 2>&1; then
  echo "   ok -- $(grep -m1 '^iterations:' "$out/ls_relacy_ok.run" || echo 'completed')"
else
  echo "   FAIL: the correct ordering did not verify"; tail -20 "$out/ls_relacy_ok.run"; fail=1
fi

echo "== relaxed state word (the control): expected to FAIL =="
if ! "$CXX" $STD -DLOCKSTEP_RELACY_RELAXED_STATE -I "$relacy" \
      -o "$out/ls_relacy_bad" "$src" 2>"$out/ls_relacy_bad.log"; then
  echo "   FAIL: control did not compile"; sed -n '1,15p' "$out/ls_relacy_bad.log"; fail=1
elif "$out/ls_relacy_bad" >"$out/ls_relacy_bad.run" 2>&1; then
  echo "   FAIL: relaxed ordering PASSED. The model is not detecting the bug it"
  echo "         exists to detect, so the run above proves nothing."
  fail=1
else
  echo "   ok -- stale payload still detected:"
  grep -m1 'NOT CURRENT' "$out/ls_relacy_bad.run" | sed 's/^/        /' || true
  grep -m1 'USER ASSERT FAILED' "$out/ls_relacy_bad.run" | sed 's/^/        /' || true
fi

exit $fail
