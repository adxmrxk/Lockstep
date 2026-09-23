#!/usr/bin/env python3
"""Judge a soak run.

The soak script records and does not decide; this decides. Keeping them apart
means the verdict can be re-derived, argued with, or recomputed with different
thresholds without re-running five hours of robot.

    python3 scripts/soak-report.py soak-20260911-000000

What it checks, and why each one is the failure it is looking for:

  memory growth    A leak is the classic thing a short test cannot see. Fitted
                   as a least-squares slope over RSS rather than first-vs-last,
                   because a single sample at either end can be noise.
  fd growth        Handles leaked per node restart. Invisible in one run,
                   fatal over a day.
  arena occupancy  Blocks handed out and never returned. Should be flat; a
                   climb means the allocator is losing them.
  overrun rate     Some overruns are normal -- it is a lossy bus. A rate that
                   ACCELERATES means a reader is falling permanently behind.
  message flow     If publishing stalls, everything else looks suspiciously
                   stable. Guards against a green report on a dead bus.
"""
import csv
import os
import sys
from collections import defaultdict


def slope(xs, ys):
    """Least-squares slope. Returns 0.0 when it cannot be computed."""
    n = len(xs)
    if n < 3:
        return 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    if den == 0:
        return 0.0
    return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den


def load(path):
    topics = defaultdict(list)   # name -> [(ts, msgs, overruns)]
    arena = defaultdict(list)    # pool -> [(ts, live, total)]
    samples = os.path.join(path, "samples.csv")
    if os.path.exists(samples):
        with open(samples) as f:
            for row in csv.reader(f):
                if not row or row[0] == "ts_ns":
                    continue
                try:
                    if row[0] == "topic" and len(row) >= 9:
                        topics[row[2]].append(
                            (int(row[1]) / 1e9, int(row[5]), int(row[8])))
                    elif row[0] == "arena" and len(row) >= 6:
                        arena[int(row[2])].append(
                            (int(row[1]) / 1e9, int(row[4]), int(row[5])))
                except (ValueError, IndexError):
                    continue

    procs = defaultdict(list)    # label -> [(ts, rss_kb, fds)]
    ppath = os.path.join(path, "procs.csv")
    if os.path.exists(ppath):
        with open(ppath) as f:
            for row in csv.reader(f):
                if not row or row[0] == "ts_s":
                    continue
                try:
                    if len(row) >= 5 and row[3]:
                        # Keyed by (label, pid), not label alone. The soak
                        # restarts nodes on purpose, so a label spans several
                        # processes; fitting one trend across all of them
                        # concatenates unrelated lifetimes into a slope that
                        # means nothing. Each generation is judged separately.
                        procs[(row[1], row[2])].append(
                            (int(row[0]), int(row[3]), int(row[4] or 0)))
                except (ValueError, IndexError):
                    continue
    return topics, arena, procs


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    path = sys.argv[1]
    if not os.path.isdir(path):
        print(f"no such soak directory: {path}")
        return 2

    topics, arena, procs = load(path)
    failures = []
    notes = []

    print(f"soak report: {path}")
    print("=" * 64)

    # ---------------------------------------------------------------- span --
    span_s = 0.0
    for series in topics.values():
        if len(series) >= 2:
            span_s = max(span_s, series[-1][0] - series[0][0])
    for series in procs.values():
        if len(series) >= 2:
            span_s = max(span_s, float(series[-1][0] - series[0][0]))
    print(f"\nduration      {span_s/3600:.2f} hours ({span_s/60:.0f} min)")
    if span_s < 60:
        failures.append("run too short to conclude anything")

    # ------------------------------------------------------------- process --
    print(f"\n{'PROCESS':<10} {'PID':>8} {'MINS':>6} {'RSS start':>10} "
          f"{'RSS end':>10} {'MB/hour':>9}  {'FDs':>10}")
    worst = {}
    for (label, pid), series in sorted(procs.items()):
        t0 = series[0][0]
        mins = (series[-1][0] - t0) / 60.0
        xs = [s[0] - t0 for s in series]
        rss = [s[1] for s in series]
        fds = [s[2] for s in series]
        fd_txt = f"{fds[0]}->{fds[-1]}"

        # A generation too short to fit cannot be judged; say so rather than
        # inventing a slope out of four points.
        if len(series) < 6 or mins < 2.0:
            print(f"{label:<10} {pid:>8} {mins:>6.1f} {rss[0]/1024:>9.1f}M "
                  f"{rss[-1]/1024:>9.1f}M {'short':>9}  {fd_txt:>10}")
            continue

        mb_per_hour = slope(xs, rss) * 3600 / 1024.0
        print(f"{label:<10} {pid:>8} {mins:>6.1f} {rss[0]/1024:>9.1f}M "
              f"{rss[-1]/1024:>9.1f}M {mb_per_hour:>9.2f}  {fd_txt:>10}")

        worst[label] = max(worst.get(label, -1e9), mb_per_hour)

        if fds[-1] > fds[0] + 10:
            failures.append(f"{label} pid {pid}: fds grew {fds[0]} -> {fds[-1]}")

    # A journalling node's RSS includes the journal's mmap pages, which fault in
    # as it records and are bounded by the journal size rather than growing
    # without limit. So the failure threshold is generous and a moderate climb
    # is a note rather than a verdict.
    for label, mb_per_hour in sorted(worst.items()):
        if mb_per_hour > 40.0:
            failures.append(
                f"{label}: RSS growing {mb_per_hour:.1f} MB/hour inside one "
                f"process -- faster than journal paging accounts for")
        elif mb_per_hour > 5.0:
            notes.append(
                f"{label}: RSS climbing {mb_per_hour:.1f} MB/hour, expected while "
                f"journalling and bounded by journal size")

    # --------------------------------------------------------------- topics --
    print(f"\n{'TOPIC':<22} {'MESSAGES':>12} {'MSG/s':>9} {'OVERRUNS':>10} {'OVR/hr':>9}")
    total_msgs = 0
    for name, series in sorted(topics.items()):
        if len(series) < 3:
            continue
        t0 = series[0][0]
        xs = [s[0] - t0 for s in series]
        msgs = [s[1] for s in series]
        ovr = [s[2] for s in series]
        total_msgs += msgs[-1]
        rate = slope(xs, msgs)
        ovr_hr = slope(xs, ovr) * 3600
        print(f"{name:<22} {msgs[-1]:>12,} {rate:>9.1f} {ovr[-1]:>10,} {ovr_hr:>9.1f}")

        if msgs[-1] <= msgs[0]:
            failures.append(f"{name}: message count did not advance")

        # LOSS RATIO, checked before anything about the rate. A steady overrun
        # rate is not automatically healthy: a reader stuck behind the trailing
        # edge overruns once per cycle forever, which is perfectly steady and
        # means it is receiving nothing. The first 5-hour run reported 339,109
        # overruns against 353,428 messages -- 96% loss -- and passed, because
        # the only check was on acceleration. Ratio catches what rate cannot.
        if msgs[-1] > 100:
            loss = ovr[-1] / float(msgs[-1])
            if loss > 0.25:
                failures.append(
                    f"{name}: losing {loss*100:.0f}% of messages "
                    f"({ovr[-1]:,} overruns / {msgs[-1]:,} published) -- "
                    f"a reader is not keeping up")
            elif loss > 0.02:
                notes.append(f"{name}: {loss*100:.1f}% of messages overrun")

        # Overruns alone are not a fault -- this is a lossy bus and a reader
        # that falls behind is supposed to lose messages and be told so. What
        # matters is whether the LOSS RATE is accelerating, which means a reader
        # is falling permanently behind rather than occasionally. Compare the
        # rate over the first half of the run against the second.
        if len(series) >= 12 and ovr[-1] > 0:
            mid = len(series) // 2
            first_span = xs[mid] - xs[0]
            second_span = xs[-1] - xs[mid]
            if first_span > 0 and second_span > 0:
                r1 = (ovr[mid] - ovr[0]) / first_span
                r2 = (ovr[-1] - ovr[mid]) / second_span
                # 2x with a floor, so a jump from 0.01/s to 0.03/s is ignored.
                if r2 > max(2.0 * r1, 1.0) and r2 > 0.5:
                    failures.append(
                        f"{name}: overrun rate accelerating, "
                        f"{r1:.2f}/s -> {r2:.2f}/s -- a reader is falling behind")
                elif r2 > 0.5:
                    notes.append(
                        f"{name}: steady overruns at {r2:.2f}/s "
                        f"(steady is expected on a lossy bus; accelerating is not)")

    if total_msgs == 0:
        failures.append("no messages published at all -- the bus was dead")

    # ---------------------------------------------------------------- arena --
    print(f"\n{'POOL':<6} {'LIVE start':>11} {'LIVE end':>10} {'OF':>8} {'drift/hr':>10}")
    for pool, series in sorted(arena.items()):
        if len(series) < 3:
            continue
        t0 = series[0][0]
        xs = [s[0] - t0 for s in series]
        live = [s[1] for s in series]
        total = series[-1][2]
        drift = slope(xs, live) * 3600
        print(f"{pool:<6} {live[0]:>11} {live[-1]:>10} {total:>8} {drift:>10.2f}")

        # Blocks are preallocated per ring slot and never freed by design, so
        # this should be flat. Any sustained climb means blocks are going out
        # and not coming back.
        if drift > 1.0:
            failures.append(
                f"arena pool {pool}: live blocks climbing {drift:.1f}/hour")

    # --------------------------------------------------------------- verdict --
    print("\n" + "=" * 64)
    for n in notes:
        print(f"  note: {n}")
    if failures:
        print(f"\nFAIL -- {len(failures)} problem(s):")
        for f in failures:
            print(f"  * {f}")
        return 1
    print("\nPASS -- no drift detected over "
          f"{span_s/3600:.2f} hours")
    return 0


if __name__ == "__main__":
    sys.exit(main())
