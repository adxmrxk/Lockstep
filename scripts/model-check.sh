#!/usr/bin/env bash
# Run TLC over the ring protocol model.
#
# Two configurations, and BOTH outcomes are asserted:
#   Ring_spmc.cfg  one publisher   -> must pass
#   Ring.cfg       two publishers  -> must FAIL, and is kept precisely because
#                                     it fails; it is the counterexample that
#                                     forced the one-publisher-per-topic rule.
set -uo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tla="$here/../docs/tla"
jar="${TLA_TOOLS_JAR:-$tla/tla2tools.jar}"

if [ ! -f "$jar" ]; then
  echo "[model-check] fetching tla2tools.jar"
  curl -sSL -o "$jar" \
    https://github.com/tlaplus/tlaplus/releases/latest/download/tla2tools.jar || {
      echo "[model-check] download failed; set TLA_TOOLS_JAR to a local copy"; exit 1; }
fi

run() {  # run <cfg> -> prints result, returns TLC's exit status
  ( cd "$tla" && java -cp "$jar" tlc2.TLC -config "$1" Ring.tla 2>&1 )
}

fail=0

echo "== Ring_spmc.cfg (one publisher): expected to PASS =="
out="$(run Ring_spmc.cfg)"
if grep -q "No error has been found" <<<"$out"; then
  echo "   ok -- $(grep -o '[0-9]* distinct states found' <<<"$out" | head -1)"
else
  echo "   FAIL: the single-publisher model did not verify"; echo "$out" | tail -20; fail=1
fi

echo "== Ring.cfg (two publishers): expected to FAIL =="
out="$(run Ring.cfg)"
if grep -q "Invariant NoTornRead is violated" <<<"$out"; then
  echo "   ok -- torn read still reproduced, in $(grep -c '^State ' <<<"$out") states"
else
  echo "   FAIL: the multi-publisher counterexample no longer reproduces."
  echo "   Either the model or the protocol changed; do not silently drop this."
  fail=1
fi

exit $fail
