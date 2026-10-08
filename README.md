# Lockstep

[![CI](https://github.com/adxmrxk/Lockstep/actions/workflows/ci.yml/badge.svg)](https://github.com/adxmrxk/Lockstep/actions/workflows/ci.yml)

**A C++20 message bus that makes robot bugs reproducible.**

Your robot does something wrong once, on a Tuesday. You replay the logs and it
behaves perfectly, because logs record what the robot *saw* and not what it
*decided, or when*. The bug never comes back and you never find it.

Lockstep records the decisions too: dispatch order, every clock read, every
dropped message. A recorded run re-executes byte for byte on your laptop, so you
get the same wrong turn on demand, as many times as you need to step through it.

Underneath it's zero-copy shared memory that keeps working when a publisher is
killed halfway through a write. That isn't the headline. It's what makes
recording *every* message cheap enough to do, and recording everything is what
makes the replay exact.

Fast pub/sub is a solved problem. Reproducibility is the part you can't get off
the shelf.

**You don't have to replace ROS 2 to use it.** A bridge node puts your existing
ROS 2 topics on the bus, so it's something you run alongside a working stack
instead of migrating to. Nobody rewrites a robot to try a message bus.

## Where this actually is

Phases 1 through 7 are built, phase 8 is partly done. 19 tests on GCC 13.3,
Clang and MSVC 19.29, 20 with a ROS 2 install. Three of them assert that code
*fails* to compile.

What's in here and running: the type layer, the shared-memory transport, the
ring protocol, pub/sub, crash consistency, the deadline executor, record and
replay. Plus a five-node demo you can watch survive a `SIGKILL`, a bridge onto a
live ROS 2 graph, a bridge onto a physical Elegoo robot car with video recording,
and `lockstep_top` for looking inside a running bus.

One thing is **not** done and I'm not implying otherwise. The real-time
*guarantee* needs `PREEMPT_RT`, isolated cores and `CAP_SYS_NICE`. This machine
has none of them, so the executor asks, reports that it was refused, and marks
its own timing as untrustworthy rather than printing a number that would mean
nothing.

Both platforms build and run everything. POSIX backend (`shm_open`/`mmap`) and
Win32 (`CreateFileMapping`), full suite on both. What's still Linux-only is the
real-time story, because `SCHED_FIFO`, `mlockall` and `PREEMPT_RT` have no
Windows equivalent worth faking.

Not compared yet: iceoryx and Fast-DDS. Cyclone DDS is, and the numbers are
below.

## The problem

Say you're moving 24 MB camera frames between processes at 30 Hz. Three things
tend to go wrong.

**DDS copies.** Fast-DDS and Cyclone serialize the message, copy it into a
transport buffer, then copy it out the other side. At 24 MB the copies alone eat
the latency budget and wreck the tail.

**Nothing reproduces.** A ROS 2 bag replays the *messages*, not the *execution*.
Callback interleaving, clock reads and thread scheduling all come out different,
so the bug you carefully recorded often just refuses to show up again.

**A crash is unbounded.** Kill a publisher mid-write and whatever shared state it
was touching stays however it was left. Subscribers block, or read a torn
message, or you tear the segment down and restart everything.

All three come back to the same substrate, so that's where Lockstep attacks
them. One shared memory segment where a message is written once in its final
location, every dispatch decision in a journal, and every slot carrying enough
ownership metadata to be reclaimed from a process that died holding it.

## How it fits together

```
 ┌──────── process A ────────┐   ┌──────── process B ────────┐
 │ camera_node               │   │ detector_node             │
 │  loan() ──┐               │   │        ▲ const view       │
 └───────────┼───────────────┘   └────────┼──────────────────┘
             ▼                            │
   ╔═════════════════ /dev/shm/bus ═══════╪═══════════════════╗
   ║  registry: topics, layout hashes, PIDs, heartbeats       ║
   ║  ring[N]: seq(atomic) │ gen │ owner_pid │ refcnt │ offset ║
   ║  arena:   fixed-size slab pools (64B … 32MB), no malloc  ║
   ╚══════════════════════════╪═══════════════════════════════╝
                              ▼
                    journal (mmap, append-only)
                      → replay engine → bit-exact re-execution
```

| # | Piece | Status |
|---|-------|--------|
| 1 | Relocatable zero-copy types | built |
| 2 | Shared memory arena + broadcast ring, model checked | built |
| 3 | Crash consistency (orphan reclamation, liveness) | built |
| 4 | Deterministic record and replay | built |
| 5 | Deadline aware executor + response time analysis | built, guarantee unproven |

## The type layer

A shared memory segment maps at a **different base address in every process**.
That one fact is why you can't put a `std::string`, a `std::vector` or any raw
pointer in a message. They all encode an address, and the address is meaningless
in the process reading it. This layer turns that class of bug into a compile
error.

`ls::offset_ptr<T>` stores a signed byte offset from its own address. Copy the
enclosing block somewhere else and both the pointer and its target move by the
same amount, so the offset is still right and nothing needs patching.

C++20 has no reflection, so there's no way to walk a struct looking for hidden
pointers. The default is flipped: class types are **not** relocatable unless you
declare them, and declaring them is what runs the checks.

```cpp
struct CameraFrame {
  std::uint64_t stamp_ns;
  std::uint32_t width;
  std::uint32_t height;
  ls::inline_string<16> frame_id;
  ls::shm_span<std::uint8_t> pixels;   // body lives in the arena
};
LOCKSTEP_MESSAGE(CameraFrame, stamp_ns, width, height, frame_id, pixels);
```

That line asserts every member is relocatable, that the type is standard layout,
trivially copyable and trivially destructible, and that your field list accounts
for **every byte of the struct**. Forget a member and it won't build. Then it
computes a 64-bit layout hash that peers trade at connect time, so a node built
against a stale header can't quietly misread a struct somebody reordered.

Three negative tests assert that bad code fails to compile, and CTest runs them
with `WILL_FAIL`. A raw pointer member, a `std::string` member, and a field list
that misses a member: all three are build failures, and if a change ever lets
one through that's a failing test.

For payloads, a 24 MB frame has no business inside a fixed-size message slot, so
`CameraFrame` stays 64 bytes and the pixels live in the arena with a span
pointing at them. `inline_vector<T,N>` and `inline_string<N>` cover the bounded
cases. All three zero their unused capacity, which is a determinism requirement
and not tidiness: a recycled slot still holding the last message's bytes would
change a replay hash taken over the whole slot.

## Build and test

CMake 3.20 or newer, and a C++20 compiler.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
```

```
100% tests passed, 0 tests failed out of 19
```

On Windows, `scripts\build.bat` and `scripts\test.bat` do the same under MSVC.
CMake sets `/Zc:preprocessor` and it isn't optional, because `LOCKSTEP_MESSAGE`
needs conformant `__VA_ARGS__` expansion and the legacy preprocessor breaks it.

The model checker is separate because it needs a JVM:

```sh
scripts/model-check.sh      # downloads tla2tools.jar, runs both configurations
```

It asserts both outcomes. The one-publisher model must verify, and the
two-publisher model must still reproduce its torn read, so a "fix" that quietly
made the counterexample stop reproducing would fail that script.

```sh
./build/examples/five_node_demo     # five processes, a SIGKILL, and a replay
./build/benchmarks/bench_latency    # latency distribution
```

`LOCKSTEP_CRASH_ITERS=10000 ./build/tests/test_crash_consistency` runs the full
ten thousand kill injections. The default of 250 keeps `ctest` under a second.

No third party libraries in the core, and I intend to keep it that way. A
message bus is the thing every other process on the robot links against, so any
dependency I take becomes a dependency for the whole fleet.

## Results

Everything here came from running the commands above on this machine. Nothing is
projected.

### Determinism

```
[replay] recorded 200 msgs, 200 clock reads, hash 6c73f0ef61858953
[replay] 1000/1000 replays matched, 0 diverged
[replay] live re-run hash f547f6c0689c6bce (differs, as it must)
```

The third line is what makes the second line mean anything. The workload
deliberately depends on a wall-clock read, so re-running it live against the same
inputs gives a *different* hash. If it didn't, replaying would be proving
nothing.

### Crash consistency

```
[crash] rounds=10000 killed=10000 had_publisher=9000
[crash] reclaimed=9000 healed=8992
[crash] accepted=577924 abandoned=61211 overruns=0 torn=0
```

10,000 real processes, each `SIGKILL`ed at a random moment while publishing.
8,992 of those kills landed between a slot being claimed and committed. Every one
was healed and skipped, and not one torn message was ever accepted.

### Latency, and against Cyclone DDS

Writer and reader in one process, timestamp immediately before the send and
immediately after the receive, exact percentiles from a sorted sample. Cyclone
0.10.5 is configured RELIABLE with `KEEP_LAST(16)` to match the ring depth. Run
it yourself with `scripts/bench-compare.sh`.

| Payload | Lockstep p50 | Cyclone p50 | Lockstep p99 | Cyclone p99 |
|---------|-------------:|------------:|-------------:|------------:|
| 32 B    | 0.09 us      | 0.44 us     | 0.12 us      | 1.98 us     |
| 64 KB   | 0.16 us      | 4.84 us     | 0.35 us      | 11.01 us    |
| 1 MB    | 0.32 us      | 417.31 us   | 0.81 us      | 812.76 us   |

The ratio at 1 MB is about 1300x, but the ratio isn't the interesting part. The
**shape** is. Across a 32,768x increase in payload Lockstep goes from 0.09 us to
0.32 us, because the payload is never copied and only a 64-byte descriptor moves.
Cyclone goes from 0.44 us to 417 us because it serializes and copies, so its
latency tracks the size of the thing being sent. That's the zero-copy argument,
measured against a real implementation instead of asserted.

Two caveats worth saying plainly. Neither column is a real-time result: on a
stock kernel the tail is the scheduler, equally for both. And at 32 B the gap is
only about 5x, because at that size neither transport is doing much. Lockstep
wins the cases it was built for and small messages aren't one of them.

### Real-time behaviour

Not measured here, and the reason is checked rather than assumed:

```
  PREREQUISITE           STATUS     VALUE
  RLIMIT_RTPRIO          MISSING    0
  PREEMPT_RT kernel      MISSING    5.15.167.4-microsoft-standard-WSL2
  RLIMIT_MEMLOCK         ok         64 MB
  SCHED_FIFO granted     MISSING    refused
  mlockall               ok         yes
```

`RLIMIT_RTPRIO` of 0 means no process here can get `SCHED_FIFO` at any priority,
root or not, and a WSL2 kernel can't bound the tail whatever priority it's given.
So `rt_validate` skips and exits 0 instead of printing a figure that wouldn't
mean anything. On a machine that qualifies, the same binary stops skipping and
starts asserting: zero deadline misses at 1 kHz, no page faults after `mlockall`,
worst-case execution inside the declared deadline, non-zero exit if any of that
fails. That's the shape the real-time claim has to take before this README makes
one.

## On a real robot

The bus runs on a computer and the robot is the body attached to it. There's a
bridge for the **Elegoo Smart Robot Car V4** in [robot/elegoo](robot/elegoo),
with full setup in [robot/elegoo/README.md](robot/elegoo/README.md).

```
[Elegoo car]                          [your computer]
  ESP32  :100  <---- WiFi / TCP ---->  elegoo_bridge
  ESP32  :80   <---- camera JPEG ---->      |
  UNO R3 (motors, ultrasonic)      robot/range  robot/camera  robot/drive
                                             |        |         ^
                                          avoider (your control code)
```

Nothing runs on the Arduino or the ESP32, they keep their stock firmware.
Lockstep needs an OS with processes and virtual memory, which is the entire
reason `offset_ptr` exists, and a 2 KB ATmega has neither.

The control node is the point. It subscribes to a distance and publishes two
motor speeds. No networking, no JSON, no knowledge that an Elegoo car exists, so
the same code drives a real car, a simulated one, or a recorded journal.

```sh
elegoo_bridge --host 192.168.4.1 --camera --duration 40000
avoider --duration 35000 --journal drive.jrnl
```

Every run drops a readable transcript in `runs/logs/run_N.txt` as it happens,
and if the camera was on, the drive as video in
`runs/videos/<date>_run_N.avi`. The transcript is flushed row by row, so a run
that gets killed still leaves everything up to the moment it died:

```
     #      TIME      SENSOR   WHAT IT DID     SPEED L/R   WHY
  ----------------------------------------------------------------------
    18     8.41s       27 cm   forward, slow     110/110   closing in
    19     8.84s       23 cm   spin right       130/-130   too close
    20    10.23s  no reading   forward, slow     110/110   sensor gave nothing
                       ^-- sensor went quiet here

  ran for              33.3 seconds, 28 decisions
  sensor answered      10 of 28 reads (36 percent)
  longest blind spell  23.1 seconds, 10.2s to 33.3s   <-- THIS IS A PROBLEM
  output hash          169357f4a8f31dbd
```

Then replay it with the car switched off and nothing moving:

```sh
avoider --replay drive.jrnl --trace
```

Same table, same summary, same hash, and the video gets rebuilt from the journal
rather than copied, because the journal is where the frames live. The frames are
recorded inputs that `decide()` never reads, so adding video can't change the
hash and doesn't weaken the reproducibility claim.

That hash matching isn't a trivial reproduction. `decide()` deliberately branches
on the clock, so which way the car turns depends on the millisecond. If replay
didn't reproduce the clock, the turns would differ and the hashes wouldn't match.

It all works with no robot too, which is how the pipeline was first verified:
`elegoo_bridge --sim` synthesizes sensors and absorbs drive commands.

### What's tested, and what needs the car

`test_elegoo` runs in CI with no hardware: every command encoder pinned to an
exact string, the motor deadband, rejection of implausible ultrasonic readings,
the socket layer against a fake robot on loopback including connect, framing,
fast failure and mid-session disconnection, the AVI muxer checked field by field,
and the journal ordering rule that keeps camera frames readable on replay.

`protocol.hpp` was originally written from Elegoo's published docs rather than a
packet capture, and it was wrong in four separate ways. All four are now fixed
against real firmware and `elegoo_bridge --probe` prints what your car actually
replies, so you can check yours in about ten seconds.

One hardware gotcha, because it cost me hours: there's an **undocumented slide
switch** on the shield near the ESP32 header. In the wrong position the WiFi
works, TCP connects, keepalives arrive, and absolutely nothing reaches the
Arduino. Everything looks healthy and no command does anything. It has to be in
the camera position.

## Alongside ROS 2

The bridge is the difference between "rewrite your robot to try this" and "run
one more process". Your ROS 2 nodes don't change:

```sh
source /opt/ros/jazzy/setup.bash
ros_bridge --in imu:/imu/data --out twist:/cmd_vel --journal run.jrnl
```

Now every IMU message your existing stack publishes is also journalled, and that
journal replays bit for bit. That's the thing `rosbag` can't do: a bag gives the
data back but not the timing, so the run diverges and the bug you were chasing
doesn't reappear. Verified against a live ROS 2 Jazzy graph driven by stock
`ros2 topic pub` at 20 Hz.

There's no generic bridge, and the reason is the premise of the project. A ROS
message is a generated class holding `std::vector` and `std::string`, both heap
pointers, both refused by `LOCKSTEP_MESSAGE`. So each bridged type gets a flat
padding-free struct and an explicit conversion. That's a real cost: a new ROS
type is a code change, not a config line. What it buys is a message that crosses
a process boundary with no serialization, no copy, and a reproducible hash on
replay. A generic bridge could offer none of that.

The conversions are lossy in places and `test_ros_bridge` pins the losses rather
than just the round trip. IMU covariance is dropped and reported as `-1`
(unknown) rather than `0`, which under REP 145 would falsely claim it's known to
be exactly zero. `Range.frame_id` is dropped and restored from a fixed frame. A
negative ROS stamp clamps to zero instead of wrapping into a year-2500 timestamp.

ROS is optional. Without a ROS 2 install on the prefix path, configure prints a
skip line and everything else builds unchanged.

## Looking inside a running bus

```sh
lockstep_top --bus lockstep-robot
```

```
lockstep top  bus topdemo   segment 1.3 MB   watching 4s

TOPIC                   PUB PID    STATE   MESSAGES    RATE/s    RING  OVERRUNS
robot/range                1145    alive        140      40.0      64         0
robot/status               1145    alive        140      40.0      16         0
robot/drive                1147    alive        140      40.0      32         0

POOL        BLOCK     IN USE       OF  USAGE
0            64 B        112      512  ###.......
1          4.0 KB          2       64  #.........
2         64.0 KB          1       40  #.........
```

Every number there already existed in the segment, so none of it costs the
publish path any bookkeeping. Rates are differenced from ring write positions
between refreshes.

It's strictly an observer. Never announces a topic, never publishes, never takes
a slot, never writes a byte. On this bus that matters more than usual, because
announcing a topic in order to inspect it would claim the exclusive publisher
slot and lock out the real publisher.

The column worth having is `STATE`. A publisher killed with `SIGKILL` shows up
immediately, before any reaper has run:

```
robot/range                1173     DEAD         72       0.0      64         0
```

which is the screen you stare at when a robot stops responding and you don't yet
know why. `--once` prints a single snapshot for scripts and CI.

## Layout

```
include/lockstep/
  core/offset_ptr.hpp       self-relative pointer, and the rules for using it
  core/relocatable.hpp      is_relocatable / Relocatable / ZeroCopyable
  core/message.hpp          LOCKSTEP_MESSAGE and message_traits
  core/layout_hash.hpp      FNV-1a over the field table, coverage check
  core/padding.hpp          padding at every depth, nested fields included
  core/clock.hpp            the one place a raw time source is read
  containers/               inline_vector, inline_string, shm_span
  shm/segment.hpp           shm_open+mmap / CreateFileMapping
  shm/arena.hpp             slab pools, tagged Treiber free list
  shm/registry.hpp          topic table and the layout-hash handshake
  shm/ring.hpp              the broadcast ring protocol
  shm/liveness.hpp          dead-publisher detection and repair
  pubsub.hpp                loan / publish / subscribe
  rt/executor.hpp           fixed-priority dispatch, allocation-free
  rt/response_time.hpp      Joseph-Pandya worst-case response time
  replay/journal.hpp        journal, deterministic clock, output hashing
  net/tcp.hpp               the only socket code
tools/lockstep_top.cpp      live read-only view of a running bus
robot/ros2/                 flat ROS equivalents, conversions, bridge node
robot/elegoo/
  protocol.hpp              the Elegoo wire format, verified against firmware
  robot_msgs.hpp            RobotRange / RobotDrive / RobotStatus / RobotFrame
  camera.hpp                pulls JPEG frames off the car's HTTP server
  avi.hpp                   MJPEG AVI muxer, so video needs no ffmpeg
  bridge.cpp                puts a physical car on the bus (--sim needs no car)
  avoider.cpp               obstacle avoidance, records and replays a drive
tests/
  test_*.cpp                plain executables, run by CTest
  negative/*.cpp            must fail to compile, CTest asserts WILL_FAIL
docs/tla/                   the ring protocol model, both configurations
examples/                   layout dump, demo node, five-node demo
benchmarks/                 latency, rt_validate, the Cyclone baseline
verification/relacy/        the memory ordering, which TLA+ can't check
```

Tests are plain executables rather than a framework, so the tree builds with
nothing but CMake and a compiler.

## Roadmap

| Phase | Work | Result |
|-------|------|--------|
| 1 | Relocatable types, layout hash, negative tests | done, 9 tests, 3 are compile-failure tests |
| 2 | Shared memory arena, slab pools, topic registry | done, two processes share a payload at different bases |
| 3 | Ring protocol, TLA+ model check | done, TLC found a torn read, one publisher per topic now enforced |
| 4 | `loan()` / `publish()` / `subscribe()` | done, zero copy proved as an address identity |
| 5 | Crash consistency, orphan reclamation, heartbeats | done, 10,000 `SIGKILL` injections, 0 corruption |
| 6 | Deadline executor, no post-init alloc | partly, 0 allocations enforced by a `new` hook, `SCHED_FIFO` refused here |
| 7 | Journal, replay, bit-exact output hashing | done, 1000 replays, 1000 matches |
| 8 | Benchmarks, 5-node demo, ROS 2 interop | partly, measured against Cyclone, ROS bridge verified on a live graph, iceoryx and Fast-DDS not compared |

Real-time numbers get measured on Linux with `PREEMPT_RT`, isolated cores and
`mlockall`. Windows and WSL2 are for getting the logic right. Timing taken there
wouldn't hold up, so I'm not publishing any.

## Prior art

[**iceoryx**](https://github.com/eclipse-iceoryx/iceoryx) already does true
zero-copy shared memory pub/sub for robotics, and does it well. I'm not trying to
beat it on throughput. What it doesn't give you is deterministic record and
replay, explicit crash-consistency guarantees, or a deadline aware executor with
analytical response time bounds, and those are the reasons this exists.

`boost::interprocess::offset_ptr` is the reference implementation of the
self-relative pointer idea. Mine keeps copy and assignment defaulted so the type
stays trivially copyable and the bus can `static_assert` that a whole message is
memcpy-able. Boost recomputes on copy instead. The price of my choice is that
lifting a single `offset_ptr` out of its block gives you a dangling pointer, so
`.get()` is the supported way out.

ROS 2 interop is a bridge node rather than a full `rmw` implementation, and that
was the right order. A bridge makes this something you add to a working stack in
an afternoon, where an `rmw` backend asks a team to swap out the layer
everything else stands on before they have any reason to trust it. The `rmw`
backend would still be nice eventually. It's still off the critical path.
