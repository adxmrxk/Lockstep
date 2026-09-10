#!/usr/bin/env bash
#
# Comparison benchmark against Cyclone DDS and iceoryx.
#
# ===========================================================================
# THIS SCRIPT HAS NEVER BEEN RUN. It is written from the documented interfaces
# of the packages below, not from a working setup, and it should be treated as
# a starting point rather than as something known to work.
#
# It could not be run on the development machine because installing the
# comparison stacks needs root, and sudo there requires a password that a
# non-interactive session cannot supply:
#
#     $ sudo -n true
#     sudo: a password is required
#
# Cyclone DDS 0.10.4 and iceoryx 2.0.5 ARE both in Ubuntu 24.04 universe, so
# this is a permissions problem and not an availability one. Fast-DDS is not
# packaged and would have to be built from source.
#
# Until someone runs this, the README must not quote a comparison number, and
# it does not.
# ===========================================================================
set -euo pipefail

if [ "$(id -u)" -ne 0 ] && ! sudo -n true 2>/dev/null; then
  echo "This needs root to install the comparison middleware." >&2
  echo "Run it as root, or configure passwordless sudo, then try again." >&2
  exit 1
fi

echo "== installing comparison middleware =="
sudo apt-get update
sudo apt-get install -y \
  libcyclonedds-dev cyclonedds-tools \
  libiceoryx-posh-dev libiceoryx-hoofs-dev iceoryx \
  || {
    echo "Package install failed. Package names differ between Ubuntu releases;" >&2
    echo "check 'apt-cache search cyclonedds iceoryx' and adjust." >&2
    exit 1
  }

echo
echo "== Lockstep baseline =="
BUILD="${BUILD_DIR:-build}"
if [ ! -x "$BUILD/examples/bench_latency" ]; then
  echo "Build Lockstep first: cmake -S . -B $BUILD && cmake --build $BUILD" >&2
  exit 1
fi
"$BUILD/examples/bench_latency" "${ITERATIONS:-100000}"

cat <<'NOTES'

== what still has to be written ==

The Lockstep side above is real and runs today. The comparison side is not
written, because writing a benchmark against a stack you cannot install is how
you end up with numbers nobody checked. What it needs:

  1. A Cyclone DDS publisher/subscriber pair over the same message shapes
     (32-byte sample, 64 KB frame, 1 MB frame), using iox shared-memory
     transport where available so the comparison is like for like.
  2. An iceoryx publisher/subscriber pair using loan/publish, which is the
     closest analogue to Lockstep's API and therefore the fairest comparison.
  3. Identical measurement discipline: same clock, timestamp immediately before
     publish and immediately after take, report the same percentiles.
  4. A PREEMPT_RT kernel with isolated cores, mlockall and CAP_SYS_NICE. On a
     stock kernel every stack under test is measuring the scheduler, and the
     comparison says nothing about the transports.

Point 4 matters more than the other three. Without it the result is not a
benchmark, it is three programs taking turns being descheduled.
NOTES
