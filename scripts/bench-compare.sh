#!/usr/bin/env bash
#
# Lockstep vs Cyclone DDS, same machine, same methodology, one run.
#
# An earlier version of this script said it had never been run and that the
# comparison needed root. That was wrong, and the mistake is worth recording:
# the APT PACKAGE needs root, but Cyclone DDS builds from source into $HOME
# perfectly happily, and that is all the comparison ever required. The
# conclusion "blocked" had been reached from `sudo -n true` failing, without
# checking whether sudo was actually necessary.
#
# Both benchmarks measure the same thing the same way: writer and reader in one
# process, timestamp taken immediately before the send and immediately after the
# receive, exact percentiles from a sorted sample vector, one-way.
#
# What this does NOT establish: real-time behaviour. On a stock kernel the tail
# is the scheduler for both, equally. Compare the two columns to each other.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$here/.."
prefix="${CDDS_PREFIX:-$HOME/cdds}"
iters="${1:-20000}"
build="${BENCH_BUILD_DIR:-$root/build-bench}"

if [ ! -d "$prefix" ]; then
  echo "[bench] Cyclone DDS not found at $prefix; building it (no root needed)"
  "$here/build-cyclone.sh" "$prefix"
fi

echo "[bench] configuring"
cmake -S "$root" -B "$build" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="$prefix" >/dev/null

echo "[bench] building"
cmake --build "$build" -j"$(nproc)" >/dev/null

if [ ! -x "$build/benchmarks/cyclone/bench_cyclone" ]; then
  echo "[bench] the Cyclone benchmark did not build; is $prefix a CycloneDDS install?" >&2
  exit 1
fi

echo
echo "############################################################"
echo "# Lockstep"
echo "############################################################"
"$build/examples/bench_latency" "$iters"

echo
echo "############################################################"
echo "# Cyclone DDS"
echo "############################################################"
"$build/benchmarks/cyclone/bench_cyclone" "$iters"

echo
echo "Both tables above came from one run on this machine. Compare them to each"
echo "other; neither is a real-time result unless the scheduling line said so."
