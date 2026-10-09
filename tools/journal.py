#!/usr/bin/env python3
"""Read a Lockstep journal from Python.

Why this exists: everyone who actually debugs robots lives in Python. A C++ bus
with no analysis story gets used only from inside the C++ process, so plotting a
drive, diffing two runs, or computing statistics across fifty journals means
writing a C++ program first. Nobody does that, so nobody looks at the data.

The format is simple enough that this needs no bindings and no build step: a
96 byte header, then records of a 32 byte header plus a payload, padded to 8.
Stdlib only. pandas is used if you have it and skipped if you do not.

Usage as a tool:

    python tools/journal.py drive.jrnl                   what is in it
    python tools/journal.py drive.jrnl --csv out.csv     range readings as CSV
    python tools/journal.py drive.jrnl --frames ./pics   pull the JPEGs out

Usage as a library:

    from journal import Journal
    j = Journal("drive.jrnl")
    for r in j:
        print(r.kind, r.sequence, r.value, len(r.payload))
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
from dataclasses import dataclass
from typing import Iterator, Optional

# Must match include/lockstep/replay/journal.hpp. If the C++ header changes its
# layout it also changes its magic, which is what keeps these two in step: a
# mismatched reader fails loudly on the magic instead of quietly misparsing.
MAGIC_V2 = 0x4C534A524E4C3032  # "LSJRNL02"
MAGIC_V1 = 0x4C534A524E4C3031  # "LSJRNL01", before ring mode

_HEADER = struct.Struct("<10Q2IQ")
HEADER_SIZE = 96
assert _HEADER.size == HEADER_SIZE

_RECORD = struct.Struct("<2I2Q2I")
RECORD_SIZE = 32
assert _RECORD.size == RECORD_SIZE

KIND_MESSAGE = 1
KIND_CLOCK = 2
KIND_OUTPUT = 3
KIND_NAMES = {KIND_MESSAGE: "message", KIND_CLOCK: "clock", KIND_OUTPUT: "output"}

MODE_LINEAR = 0
MODE_RING = 1
MODE_NAMES = {MODE_LINEAR: "linear", MODE_RING: "ring"}


def _align8(n: int) -> int:
    return (n + 7) & ~7


@dataclass(frozen=True)
class Record:
    kind: int
    callback: int
    sequence: int
    value: int
    payload: bytes

    @property
    def kind_name(self) -> str:
        return KIND_NAMES.get(self.kind, "kind%d" % self.kind)


# Payload decoders for the message types that ship with the project.
#
# Keyed by size, because a journal records a message's bytes and not its type:
# the layout hash lives in the bus registry, not in the record. That is a real
# limitation and worth saying out loud rather than pretending otherwise. Size
# plus the callback id is enough to tell apart the handful of types any one node
# records, which is what this is for.

_ROBOT_RANGE = struct.Struct("<QfIII")  # stamp_ns, distance_cm, seq, valid, pad


def decode_robot_range(p: bytes) -> Optional[dict]:
    """robot/elegoo/robot_msgs.hpp RobotRange, 24 bytes."""
    if len(p) != 24:
        return None
    stamp, dist, seq, valid, _ = _ROBOT_RANGE.unpack(p)
    return {
        "stamp_ns": stamp,
        "distance_cm": dist if valid else None,
        "seq": seq,
        "valid": bool(valid),
    }


def is_jpeg(p: bytes) -> bool:
    return len(p) >= 3 and p[0] == 0xFF and p[1] == 0xD8


class Journal:
    """A Lockstep journal, read-only.

    Iterating yields Records oldest first. Ring journals are handled: the
    on-disk layout keeps two halves and the older one is not always first, so
    iteration walks them in age order the same way the C++ reader does.
    """

    def __init__(self, path: str):
        self.path = path
        with open(path, "rb") as f:
            self._buf = f.read()
        if len(self._buf) < HEADER_SIZE:
            raise ValueError("%s is too short to be a journal" % path)

        (
            magic,
            self.capacity,
            self.used,
            self.records,
            self.dropped,
            self.half,
            a_bytes,
            b_bytes,
            a_records,
            b_records,
            mode,
            self.active,
            self.rotations,
        ) = _HEADER.unpack_from(self._buf, 0)

        if magic == MAGIC_V1:
            raise ValueError(
                "%s is a v1 journal, recorded before ring mode. Re-record it." % path
            )
        if magic != MAGIC_V2:
            raise ValueError("%s is not a Lockstep journal" % path)

        self.mode = mode
        self.half_bytes = (a_bytes, b_bytes)
        self.half_records = (a_records, b_records)
        self._area = HEADER_SIZE

    @property
    def mode_name(self) -> str:
        return MODE_NAMES.get(self.mode, "mode%d" % self.mode)

    def _physical(self, logical: int) -> int:
        """Offset in the live window, to offset in the record area.

        Mirrors journal::physical(). Linear mode is the identity. In ring mode
        the older half comes first, and that is the half that is NOT active once
        anything has rotated; before the first rotation only half 0 was ever
        written, so there is nothing to put in front of it.
        """
        if self.half == 0:
            return logical
        first = (self.active ^ 1) if self.rotations > 0 else self.active
        first_bytes = self.half_bytes[first]
        if logical < first_bytes:
            return first * self.half + logical
        return (first ^ 1) * self.half + (logical - first_bytes)

    def __iter__(self) -> Iterator[Record]:
        cursor = 0
        while cursor < self.used:
            at = self._area + self._physical(cursor)
            kind, callback, sequence, value, nbytes, _ = _RECORD.unpack_from(
                self._buf, at
            )
            start = at + RECORD_SIZE
            payload = self._buf[start : start + nbytes] if nbytes else b""
            yield Record(kind, callback, sequence, value, payload)
            cursor += _align8(RECORD_SIZE + nbytes)

    def __len__(self) -> int:
        return self.records

    def messages(self, callback: Optional[int] = None) -> Iterator[Record]:
        for r in self:
            if r.kind != KIND_MESSAGE:
                continue
            if callback is not None and r.callback != callback:
                continue
            yield r

    def clocks(self) -> Iterator[Record]:
        return (r for r in self if r.kind == KIND_CLOCK)

    def summary(self) -> dict:
        """Counts and spans, in one pass."""
        by_kind: dict[str, int] = {}
        by_callback: dict[int, int] = {}
        payload_sizes: dict[int, int] = {}
        jpegs = 0
        jpeg_bytes = 0
        first_clock = None
        last_clock = None

        for r in self:
            by_kind[r.kind_name] = by_kind.get(r.kind_name, 0) + 1
            if r.kind == KIND_MESSAGE:
                by_callback[r.callback] = by_callback.get(r.callback, 0) + 1
                payload_sizes[len(r.payload)] = (
                    payload_sizes.get(len(r.payload), 0) + 1
                )
                if is_jpeg(r.payload):
                    jpegs += 1
                    jpeg_bytes += len(r.payload)
            elif r.kind == KIND_CLOCK:
                if first_clock is None:
                    first_clock = r.value
                last_clock = r.value

        span_s = None
        if first_clock is not None and last_clock is not None and last_clock > first_clock:
            span_s = (last_clock - first_clock) / 1e9

        return {
            "path": self.path,
            "mode": self.mode_name,
            "records": self.records,
            "dropped": self.dropped,
            "rotations": self.rotations,
            "used_bytes": self.used,
            "capacity_bytes": self.capacity,
            "by_kind": by_kind,
            "by_callback": by_callback,
            "payload_sizes": payload_sizes,
            "jpeg_frames": jpegs,
            "jpeg_bytes": jpeg_bytes,
            "clock_span_s": span_s,
        }

    def ranges(self, callback: int = 0) -> list:
        """Every 24 byte payload decoded as a RobotRange."""
        out = []
        for r in self.messages(callback=callback):
            d = decode_robot_range(r.payload)
            if d is not None:
                out.append(d)
        return out

    def to_dataframe(self, callback: int = 0):
        """Range readings as a pandas DataFrame, with a seconds column.

        Raises ImportError with something useful if pandas is missing, rather
        than failing at import time and taking the rest of the module with it.
        """
        try:
            import pandas as pd
        except ImportError as e:
            raise ImportError(
                "to_dataframe needs pandas (pip install pandas). "
                "Journal.ranges() returns plain dicts and needs nothing."
            ) from e
        rows = self.ranges(callback=callback)
        df = pd.DataFrame(rows)
        if not df.empty:
            df["t_s"] = (df["stamp_ns"] - df["stamp_ns"].iloc[0]) / 1e9
        return df

    def extract_frames(self, out_dir: str, callback: Optional[int] = None) -> int:
        """Write every JPEG payload out as a numbered file.

        Numbered by position in the journal, so frame 19 is the 20th frame
        recorded and lines up with the decision tables the avoider prints.
        """
        os.makedirs(out_dir, exist_ok=True)
        n = 0
        for r in self.messages(callback=callback):
            if not is_jpeg(r.payload):
                continue
            with open(os.path.join(out_dir, "%05d.jpg" % n), "wb") as f:
                f.write(r.payload)
            n += 1
        return n


def _print_summary(j: Journal) -> None:
    s = j.summary()
    print("%s" % s["path"])
    print("  mode            %s" % s["mode"])
    print(
        "  records         %d live%s"
        % (
            s["records"],
            ", %d dropped by rotation" % s["dropped"] if s["dropped"] else "",
        )
    )
    if s["rotations"]:
        print("  rotations       %d" % s["rotations"])
    print(
        "  size            %.1f MB used of %.1f MB"
        % (s["used_bytes"] / 1e6, s["capacity_bytes"] / 1e6)
    )
    if s["clock_span_s"] is not None:
        print("  clock span      %.2f s" % s["clock_span_s"])
    print("  by kind         %s" % ", ".join(
        "%s %d" % (k, v) for k, v in sorted(s["by_kind"].items())))
    if s["by_callback"]:
        print("  by callback     %s" % ", ".join(
            "%d: %d" % (k, v) for k, v in sorted(s["by_callback"].items())))
    if s["payload_sizes"]:
        sizes = sorted(s["payload_sizes"].items(), key=lambda kv: -kv[1])[:5]
        print("  payload sizes   %s" % ", ".join(
            "%dB x%d" % (k, v) for k, v in sizes))
    if s["jpeg_frames"]:
        print(
            "  video           %d JPEG frames, %.1f MB"
            % (s["jpeg_frames"], s["jpeg_bytes"] / 1e6)
        )

    rows = j.ranges()
    if rows:
        good = [r for r in rows if r["valid"]]
        print("  range readings  %d, %d valid (%.0f percent)"
              % (len(rows), len(good), 100.0 * len(good) / len(rows)))
        if good:
            lo = min(r["distance_cm"] for r in good)
            hi = max(r["distance_cm"] for r in good)
            print("  distance        %.0f to %.0f cm" % (lo, hi))


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(description="Read a Lockstep journal.")
    ap.add_argument("path")
    ap.add_argument("--csv", metavar="FILE",
                    help="write decoded range readings as CSV")
    ap.add_argument("--frames", metavar="DIR",
                    help="write every JPEG payload into DIR")
    ap.add_argument("--callback", type=int, default=None,
                    help="only this callback id")
    ap.add_argument("--dump", type=int, metavar="N", default=0,
                    help="print the first N records")
    args = ap.parse_args(argv)

    try:
        j = Journal(args.path)
    except (OSError, ValueError) as e:
        print("error: %s" % e, file=sys.stderr)
        return 1

    _print_summary(j)

    if args.dump:
        print()
        print("  %5s %-8s %4s %12s %10s" % ("#", "KIND", "CB", "SEQ", "PAYLOAD"))
        for i, r in enumerate(j):
            if i >= args.dump:
                break
            print("  %5d %-8s %4d %12d %10s"
                  % (i, r.kind_name, r.callback, r.sequence,
                     "%d B" % len(r.payload) if r.payload else "-"))

    if args.csv:
        rows = j.ranges(callback=args.callback or 0)
        if not rows:
            print("no decodable range readings, nothing written",
                  file=sys.stderr)
        else:
            t0 = rows[0]["stamp_ns"]
            with open(args.csv, "w", encoding="utf-8") as f:
                f.write("t_s,stamp_ns,seq,valid,distance_cm\n")
                for r in rows:
                    f.write("%.6f,%d,%d,%d,%s\n" % (
                        (r["stamp_ns"] - t0) / 1e9, r["stamp_ns"], r["seq"],
                        1 if r["valid"] else 0,
                        "" if r["distance_cm"] is None else "%.1f" % r["distance_cm"]))
            print("\nwrote %d rows to %s" % (len(rows), args.csv))

    if args.frames:
        n = j.extract_frames(args.frames, callback=args.callback)
        print("\nwrote %d frames to %s" % (n, args.frames))
        if n:
            print("make a video with:  ffmpeg -framerate 12 -i %s/%%05d.jpg out.mp4"
                  % args.frames)

    return 0


if __name__ == "__main__":
    sys.exit(main())
