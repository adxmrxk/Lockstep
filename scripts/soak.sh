#!/usr/bin/env bash
# Run the bus for hours and record whether anything drifts.
#
# The longest this project had ever run in one go was about eight seconds. That
# is enough to show the logic is right and tells you nothing about whether it
# stays right. A soak is the cheapest way to find the failures that only appear
# with time: memory that creeps, blocks that are never returned, handles leaked
# on every node restart, latency that degrades as a ring ages.
#
#   scripts/soak.sh --hours 5 --out soak-run
#
# WHAT IT DOES
#   * runs a realistic graph -- the robot bridge in simulation plus the avoider
#     control node -- rather than a synthetic publish loop, because the point is
#     to exercise the code paths a real deployment uses;
#   * samples every 10s into CSV: RSS and fd count per process, plus every
#     topic's message count, rate and overruns, plus arena occupancy;
#   * restarts the control node periodically, so node churn and the reaper are
#     exercised over hours rather than in a tight test loop;
#   * writes everything to <out>/ for scripts/soak-report.py to judge.
#
# WHAT IT DOES NOT DO
#   It does not pass or fail on its own. Deciding what counts as drift is the
#   report's job, so the raw data stays raw and the verdict is re-derivable.
#
# A NOTE ON WSL2. Windows can suspend the VM, which shows up in the data as a
# clock jump and a latency cliff that look like bugs and are not. If this is the
# only machine available, leave it awake and treat a single huge outlier as
# suspect rather than real.
set -uo pipefail

HOURS=5
INTERVAL=10
OUT="soak-$(date +%Y%m%d-%H%M%S)"
BUS="soak"
RESTART_EVERY=600   # seconds; 0 disables node churn
RATE=20

usage() {
  cat <<'EOF'
scripts/soak.sh -- run the bus for hours and record whether anything drifts

  --hours <n>       run length in hours (default 5, fractions allowed)
  --interval <s>    sample period in seconds (default 10)
  --out <dir>       output directory (default soak-<timestamp>)
  --bus <name>      segment name (default soak)
  --rate <hz>       sensor rate (default 20)
  --restart <s>     restart the control node every N seconds, 0 to disable
                    (default 600)
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --hours)    HOURS="$2"; shift 2 ;;
    --interval) INTERVAL="$2"; shift 2 ;;
    --out)      OUT="$2"; shift 2 ;;
    --bus)      BUS="$2"; shift 2 ;;
    --rate)     RATE="$2"; shift 2 ;;
    --restart)  RESTART_EVERY="$2"; shift 2 ;;
    -h|--help)  usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
root="$here/.."

# Find the build. Prefer an explicit BUILD, else the usual spots.
BUILD="${BUILD:-}"
if [ -z "$BUILD" ]; then
  for cand in "$root/b" "$root/build" "$HOME/lsros/b" "$HOME/lst3/b"; do
    [ -x "$cand/robot/elegoo/elegoo_bridge" ] && BUILD="$cand" && break
  done
fi
if [ -z "$BUILD" ] || [ ! -x "$BUILD/robot/elegoo/elegoo_bridge" ]; then
  echo "soak: cannot find a build with elegoo_bridge. Set BUILD=<dir>." >&2
  exit 1
fi

BRIDGE="$BUILD/robot/elegoo/elegoo_bridge"
AVOIDER="$BUILD/robot/elegoo/avoider"
TOP="$BUILD/tools/lockstep_top"
for exe in "$BRIDGE" "$AVOIDER" "$TOP"; do
  [ -x "$exe" ] || { echo "soak: missing $exe" >&2; exit 1; }
done

mkdir -p "$OUT"
SAMPLES="$OUT/samples.csv"
PROCS="$OUT/procs.csv"
EVENTS="$OUT/events.log"

DURATION_S=$(awk -v h="$HOURS" 'BEGIN{printf "%d", h*3600}')
DURATION_MS=$((DURATION_S * 1000))

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$EVENTS"; }

echo "ts_ns,kind,a,b,c,d,e" > "$SAMPLES"
echo "ts_s,proc,pid,rss_kb,fds,threads" > "$PROCS"

log "soak starting: ${HOURS}h (${DURATION_S}s), sampling every ${INTERVAL}s"
log "build: $BUILD"
log "output: $OUT"

cleanup() {
  log "stopping"
  [ -n "${BRIDGE_PID:-}" ] && kill "$BRIDGE_PID" 2>/dev/null
  [ -n "${AVOID_PID:-}" ] && kill "$AVOID_PID" 2>/dev/null
  sleep 1
  kill -9 "${BRIDGE_PID:-}" "${AVOID_PID:-}" 2>/dev/null
}
trap cleanup EXIT INT TERM

# The bridge owns the bus and runs for the whole soak.
"$BRIDGE" --sim --bus "$BUS" --rate "$RATE" --duration "$DURATION_MS" \
  > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 2

start_avoider() {
  # A fresh journal per generation. 256 MB rather than the 32 MB default: at
  # ~1.9 KB/s a 32 MB journal fills in 4.8 hours, which would truncate a 5-hour
  # recording in its final minutes and make the result a lie.
  "$AVOIDER" --bus "$BUS" --duration "$DURATION_MS" \
    --journal "$OUT/drive-$(date +%s).jrnl" > "$OUT/avoider.log" 2>&1 &
  AVOID_PID=$!
  log "avoider started, pid $AVOID_PID"
}
start_avoider

proc_sample() {  # proc_sample <label> <pid> <ts>
  local label="$1" pid="$2" ts="$3"
  [ -d "/proc/$pid" ] || { echo "$ts,$label,$pid,,," >> "$PROCS"; return; }
  local rss fds threads
  rss=$(awk '/VmRSS/{print $2}' "/proc/$pid/status" 2>/dev/null)
  fds=$(ls "/proc/$pid/fd" 2>/dev/null | wc -l)
  threads=$(awk '/Threads/{print $2}' "/proc/$pid/status" 2>/dev/null)
  echo "$ts,$label,$pid,${rss:-},${fds:-},${threads:-}" >> "$PROCS"
}

END=$(( $(date +%s) + DURATION_S ))
LAST_RESTART=$(date +%s)
SAMPLE_N=0

while [ "$(date +%s)" -lt "$END" ]; do
  NOW=$(date +%s)

  # Bus-side metrics, straight from the segment.
  "$TOP" --bus "$BUS" --once --csv --interval 1000 2>/dev/null >> "$SAMPLES"

  proc_sample bridge "$BRIDGE_PID" "$NOW"
  proc_sample avoider "$AVOID_PID" "$NOW"

  SAMPLE_N=$((SAMPLE_N + 1))
  if [ $((SAMPLE_N % 30)) -eq 0 ]; then
    REMAIN=$(( (END - NOW) / 60 ))
    log "sample $SAMPLE_N, ${REMAIN} min remaining"
  fi

  # Node churn: kill and restart the control node so the reaper, the topic
  # takeover path and fd handling are exercised across the whole run rather
  # than only at startup.
  if [ "$RESTART_EVERY" -gt 0 ] && [ $((NOW - LAST_RESTART)) -ge "$RESTART_EVERY" ]; then
    log "restarting avoider (pid $AVOID_PID) -- node churn"
    kill -9 "$AVOID_PID" 2>/dev/null
    wait "$AVOID_PID" 2>/dev/null
    sleep 1
    start_avoider
    LAST_RESTART=$NOW
  fi

  # If the bridge died the soak is over and that is itself the finding.
  if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    log "BRIDGE DIED at sample $SAMPLE_N -- soak ends early"
    break
  fi

  sleep "$INTERVAL"
done

log "soak finished after $SAMPLE_N samples"
log "report with: python3 scripts/soak-report.py $OUT"
