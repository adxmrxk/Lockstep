# Lockstep

[![CI](https://github.com/adxmrxk/Lockstep/actions/workflows/ci.yml/badge.svg)](https://github.com/adxmrxk/Lockstep/actions/workflows/ci.yml)

**A C++20 message bus that makes robot bugs reproducible.**

A robot does something wrong once, on a Tuesday. You replay the logs and it
behaves perfectly, because logs record what the robot *saw* and not what it
*decided, or when*. The bug never comes back and you never find it.

Lockstep records the decisions too -- dispatch order, every clock read, every
dropped message -- so a recorded run re-executes byte for byte on your laptop.
The same wrong turn, on demand, as many times as you need to step through it.

Underneath, the transport is zero-copy shared memory that keeps working when a
publisher is killed halfway through a write. That is not the headline feature --
it is what makes recording *every* message affordable, and recording everything
is what makes the replay exact.

Fast pub/sub is a solved problem. Reproducibility is the part you cannot get off
the shelf.

**You do not have to replace ROS 2 to use it.** A bridge node puts your existing
ROS 2 topics on the bus, so this is something you run *alongside* a working
stack rather than something you migrate to. Nobody rewrites a robot to try a
message bus, and they should not have to.

> **Where this actually is: phases 1-7 built, phase 8 partly.** 19 tests on
> GCC 13.3 and MSVC 19.29, 20 with a ROS 2 install, three of which assert that
> code *fails* to compile. The type layer, the shared-memory transport, the ring
> protocol, pub/sub, crash consistency, the deadline executor and record/replay
> are in this repo and run. There is a five-node demo you can watch survive a
> `SIGKILL`, a bridge onto a live ROS 2 graph, a bridge onto a physical Elegoo
> robot car, and `lockstep_top` for looking inside a running bus.
>
> One thing is **not** done and is not implied to be. The real-time
> *guarantee* needs `PREEMPT_RT`, isolated cores and `CAP_SYS_NICE`, none of
> which this machine has — the executor asks for them, reports that it was
> refused, and marks its own timing as untrustworthy.
>
> The Cyclone DDS comparison **has** now been run, and the numbers are below.
> An earlier version of this README said it could not be, because installing
> Cyclone needs root. That was wrong: the *apt package* needs root, and
> building from source into `$HOME` does not. iceoryx and Fast-DDS are still
> not compared.
>
> **Both platforms build and run everything.** The transport has a POSIX
> backend (`shm_open`/`mmap`) and a Win32 one (`CreateFileMapping`), and the
> multi-process tests run on both -- 18 of 18 on GCC 13.3 and on MSVC 19.29.
> What is still Linux-only is the real-time story: `SCHED_FIFO`, `mlockall`
> and `PREEMPT_RT` have no Windows equivalent worth faking, and the executor
> reports that it did not get them rather than pretending otherwise.

## Contents

- [The problem](#the-problem)
- [How it fits together](#how-it-fits-together)
- [Phase 1: the type layer](#phase-1-the-type-layer)
- [What won't compile](#what-wont-compile)
- [Tech stack](#tech-stack)
- [Build and test](#build-and-test)
- [Results](#results)
- [On a real robot](#on-a-real-robot)
- [Alongside ROS 2](#alongside-ros-2)
- [Looking inside a running bus](#looking-inside-a-running-bus)
- [Layout](#layout)
- [Roadmap](#roadmap)
- [Prior art](#prior-art)

## The problem

Say you have a perception to control stack moving 24 MB camera frames between
processes at 30 Hz. Three things tend to go wrong.

**DDS copies.** Fast-DDS and Cyclone serialize the message, copy it into a
transport buffer, then copy it out the other side. At 24 MB the copies alone eat
the latency budget and wreck the tail.

**Nothing reproduces.** A ROS 2 bag replays the *messages*. It does not replay
the *execution*. Callback interleaving, wall clock reads and thread scheduling
all come out different, so the bug you carefully recorded often just refuses to
show up again.

**A crash is unbounded.** Kill a publisher mid-write and whatever shared state
it was touching stays however it was left. Subscribers block, or read a torn
message, or you tear the whole segment down and restart everything.

All three come back to the same substrate, so that's where Lockstep attacks
them: one shared memory segment where a message is written once in its final
location, every dispatch decision goes into a journal, and every slot carries
enough ownership metadata to be reclaimed from a process that died holding it.

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

Five pieces, in the order they have to be built:

| # | Piece | Status |
|---|-------|--------|
| 1 | Relocatable zero-copy types | **built** |
| 2 | Shared memory arena + broadcast ring, model checked | **built** |
| 3 | Crash consistency (orphan reclamation, liveness) | **built** |
| 4 | Deterministic record and replay | **built** |
| 5 | Deadline aware executor + response time analysis | **built**, guarantee unproven |

## Phase 1: the type layer

A shared memory segment maps at a **different base address in every process**.
That one fact is why you can't just put a `std::string` or a `std::vector` or
any raw pointer into a message. Those all encode an address, and the address is
meaningless in the process reading it. This whole layer exists to turn that
class of bug into a compile error instead of a Tuesday afternoon.

### `ls::offset_ptr<T>`

Stores a signed byte offset from its own address instead of an absolute address.
Copy the enclosing block somewhere else and both the pointer and its target move
by the same amount, so the offset is still right and the bytes never need
patching:

```cpp
std::memcpy(dst_block, src_block, block_size);   // all internal links survive
```

One deliberate difference from `boost::interprocess::offset_ptr`: copy and
assignment are defaulted here, which keeps `offset_ptr` trivially copyable so
the bus can `static_assert` that a whole message is memcpy-able and then treat
it as bytes. Boost recomputes on copy instead. The price of my choice is that
lifting a single `offset_ptr` out of its block gives you a dangling pointer, so
`.get()` is the supported way out.

The other rule is that an `offset_ptr` and whatever it points at have to sit in
the same relocatable block. Relocation copies a block's bytes elsewhere, so an
offset reaching past the end of the block will land on whatever happens to be at
that address in the destination. The arena enforces this once it exists.

### `LOCKSTEP_MESSAGE`

C++20 has no reflection, so there is no way to walk a struct looking for hidden
pointers. So the default is flipped: class types are **not** relocatable unless
you declare them, and declaring them is what runs the checks.

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

That one line asserts every member is relocatable, that the type is standard
layout, trivially copyable and trivially destructible, and that your field list
**accounts for every byte of the struct** (forget a member and it won't build).
Then it computes a 64-bit layout hash that the two peers will trade at connect
time, so a node built against a stale header can't quietly misread a struct
somebody reordered.

```
$ ./example_message_layout
CameraFrame
  size 64  align 8  fields 7  padding yes
  layout hash 0x0e97c4d55ccc40c7
  off    size   field          type
  0      8      stamp_ns       unsigned __int64
  8      4      width          unsigned int
  ...
  48     16     pixels         struct ls::shm_span<unsigned char>
```

That `padding yes` matters more than it looks. Padding bytes are indeterminate,
so any message that has some has to be zero filled before publish, otherwise a
replay hash taken over the slot won't reproduce. The traits report it so the
publisher can enforce it.

### Containers

| Type | Storage | Good for |
|------|---------|----------|
| `inline_vector<T, N>` | inline, fixed capacity | bounded lists inside a message |
| `inline_string<N>` | inline, fixed capacity | frame ids, sensor names |
| `shm_span<T>` | `offset_ptr` + length | big payloads out in the arena |

A 24 MB frame has no business sitting inside a fixed size message slot, so
`CameraFrame` stays **64 bytes** and the pixels live in the arena with a span
pointing at them.

All three zero out their unused capacity instead of just resetting a length.
That's a determinism requirement rather than tidiness: a recycled slot still
holding the previous message's bytes would change a replay hash taken over the
whole slot.

## What won't compile

Three negative tests assert that the following code **fails to build**. CTest
runs them with `WILL_FAIL`, so if a change ever lets one of them through, that's
a failing test.

```cpp
struct HasRawPointer { std::uint64_t stamp; const std::uint8_t* pixels; };
LOCKSTEP_MESSAGE(HasRawPointer, stamp, pixels);
```
```
error C2338: LOCKSTEP_MESSAGE(HasRawPointer): a member is not relocatable. Raw
pointers, references, std::string and std::vector cannot cross a shared-memory
boundary -- they encode addresses that are meaningless in another process. Use
ls::offset_ptr, ls::inline_string<N>, ls::inline_vector<T, N> or ls::shm_span<T>.
```

```cpp
struct MissingAField { std::uint32_t a, b, forgotten; };
LOCKSTEP_MESSAGE(MissingAField, a, b);        // forgot one
```
```
error C2338: LOCKSTEP_MESSAGE(MissingAField): the field list does not account
for every byte of the struct. List every member, in declaration order.
```

A `std::string` member manages to trip three assertions at once: not
relocatable, not trivially copyable, not trivially destructible.

## Tech stack

Every row below is marked either **in use**, meaning it is in the repo today and
running, or **planned**, meaning it is not written yet and I have put the phase
number next to it. Nothing here is aspirational without being labelled as such.

### In use today

Phase 1 is header only with no third party libraries at all, and that is on
purpose. A message bus is the thing every other process on the robot links
against, so any dependency I take becomes a dependency for the whole fleet.

| Component | Status | Detail |
|-----------|--------|--------|
| **Language** | in use | C++20. Concepts for the type contracts, `std::launder` inside `offset_ptr`, fold expressions for the field checks, constexpr `string_view` for the layout hash. |
| **Build** | in use | CMake 3.20+. Generator agnostic, though I drive it with NMake Makefiles since the VS generator cannot see my Build Tools install. |
| **Compiler** | in use | MSVC 19.29 (VS 2019 Build Tools, x64), with `/std:c++20 /Zc:preprocessor /permissive- /W4`. |
| **Testing** | in use | CTest driving plain executables. No framework, so the tree builds with just CMake and a compiler. Negative tests use `WILL_FAIL` on the build itself. |
| **Metaprogramming** | in use | A generated 24 arity preprocessor `FOR_EACH`, `__FUNCSIG__` / `__PRETTY_FUNCTION__` scraping for compile time type names, and constexpr FNV-1a for the layout hash. |
| **Third party deps** | in use | None, and I intend to keep it that way for the core. |
| **Shared memory** | in use, phase 2 | POSIX `shm_open` + `mmap`, and Win32 `CreateFileMapping` + `MapViewOfFile`. Both backends run the full suite, multi-process tests included. |
| **Lock free primitives** | in use, phase 3 | `std::atomic` with explicit orders: a tagged Treiber stack for the arena free list, a seqlock per ring slot. |
| **Formal verification** | in use, phase 3 | TLA+/TLC for the protocol (found a torn read a 60k-message stress test never produced), Relacy for the C++11 memory ordering, and ThreadSanitizer against the real implementation (found two more). Each keeps a deliberately-broken control config that is asserted to fail. CDSChecker is not used; Relacy covers the same ground. |
| **Crash consistency** | in use, phase 5 | `pidfd_open` with a `kill(pid, 0)` fallback, plus a reaper that heals abandoned ring slots. |
| **GCC / Clang** | in use | GCC 13.3 is the primary toolchain now; MSVC still builds phase 1. Clang is still untried. |

### Planned, by phase

Most of what is left is platform API rather than libraries, which is the point.
None of this is in the repo yet.

| Component | Status | Detail |
|-----------|--------|--------|
| **Real time scheduling** | requested, refused here | `mlockall` works. `SCHED_FIFO` needs a non-zero `RLIMIT_RTPRIO` (this machine: 0) and the guarantee needs `PREEMPT_RT` (this machine: `5.15.167.4-microsoft-standard-WSL2`). `rt_validate` prints exactly which prerequisite is missing and skips rather than reporting a meaningless number; on an eligible box it asserts on jitter, deadline misses and page faults. |
| **Benchmarking** | mostly done | Lockstep's own latency distribution, and a Cyclone DDS 0.10.5 baseline measured the same way in the same run — `scripts/bench-compare.sh` builds Cyclone into `$HOME` (no root) and runs both. iceoryx and Fast-DDS are still not compared. |
| **Memory-model checking** | still planned, phase 3 | CDSChecker or relacy for the C++11 model, ThreadSanitizer against the implementation. The TLA+ model is sequentially consistent, so it does not cover the barriers. |
| **ROS 2 interop** | in use | An `rclcpp` bridge node carrying `sensor_msgs/Imu`, `sensor_msgs/Range` and `geometry_msgs/Twist`. Verified against a live ROS 2 Jazzy graph. A full `rmw` backend is still a stretch goal and still off the critical path. |
| **CI** | in use | GitHub Actions: Linux GCC and Clang, Windows MSVC, TSan, ASan+UBSan, the TLA+ model check, and the 10k SIGKILL soak. The suite is repeated five times per run, because three of this project bugs only showed on repetition. |

## Build and test

You need CMake 3.20 or newer and a C++20 compiler.

**Linux (everything):**

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
```

```
100% tests passed, 0 tests failed out of 19
```

**Windows:** `scripts\build.bat` and `scripts\test.bat` build and run the
whole suite under MSVC, 18 of 18, including the two-process and crash tests.
The Win32 backend maps a page-file-backed section rather than a `/dev/shm`
object, so there is no name to unlink and nothing stale left behind after a
crash -- `segment::unlink` is a documented no-op there.

CMake sets `/Zc:preprocessor` on MSVC and it is not optional. `LOCKSTEP_MESSAGE`
needs conformant `__VA_ARGS__` expansion and the legacy preprocessor breaks it.

**The model checker** is separate, because it needs a JVM:

```sh
scripts/model-check.sh      # downloads tla2tools.jar, runs both configurations
```

It asserts both outcomes: the one-publisher model must verify, and the
two-publisher model must still reproduce its torn read. A "fix" that silently
made the counterexample stop reproducing would fail that script.

**The demo and the benchmark:**

```sh
./build/examples/five_node_demo     # five processes, a SIGKILL, and a replay
./build/examples/bench_latency      # latency distribution
```

**Longer runs.** `LOCKSTEP_CRASH_ITERS=10000 ./build/tests/test_crash_consistency`
runs the full ten thousand kill injections; the default of 250 keeps `ctest`
under a second.

## Results

Everything here was produced by running the commands above on this machine.
Nothing is projected.

### Determinism (phase 7)

```
[replay] recorded 200 msgs, 200 clock reads, hash 6c73f0ef61858953
[replay] 1000/1000 replays matched, 0 diverged
[replay] live re-run hash f547f6c0689c6bce (differs, as it must)
```

The third line is what makes the second line mean anything. The workload
deliberately depends on a wall-clock read, so re-running it live against the
same inputs produces a *different* hash. If it did not, replaying would be
proving nothing and the test says so in those words.

### Crash consistency (phase 5)

```
[crash] rounds=10000 killed=10000 had_publisher=9000
[crash] reclaimed=9000 healed=8992
[crash] accepted=577924 abandoned=61211 overruns=0 torn=0
```

10,000 real processes, each `SIGKILL`ed at a random moment while publishing.
8,992 of those kills landed between a slot being claimed and committed. Every
one was healed and skipped; not one torn message was ever accepted.

### Latency

**These are not real-time numbers.** `SCHED_FIFO` was refused on this machine,
so the tail below is the scheduler, not the bus. They are here because they
demonstrate one specific thing well:

| payload | p50 | p99 | p99.9 |
|---------|-----|-----|-------|
| 32 B, inline | 0.09 us | 0.11 us | 0.13 us |
| 64 KB, out-of-line | 0.16 us | 0.51 us | 0.94 us |
| 1 MB, out-of-line | 0.30 us | 0.95 us | 2.59 us |

Latency barely moves as the payload grows by five orders of magnitude, because
nothing copies the payload. A transport that serialised and copied would show
this scaling linearly with size — a 1 MB `memcpy` alone is tens of microseconds.
That flatness is the zero-copy claim, measured.

### Against Cyclone DDS

One run, one machine, same methodology on both sides: writer and reader in a
single process, timestamp taken immediately before the send and immediately
after the receive, exact percentiles from a sorted sample vector. Cyclone is
configured RELIABLE with `KEEP_LAST(16)`, matching the ring depth Lockstep uses.
Run it yourself with `scripts/bench-compare.sh`.

| Payload | Lockstep p50 | Cyclone DDS p50 | Lockstep p99 | Cyclone p99 |
|---------|-------------:|----------------:|-------------:|------------:|
| 32 B    | 0.09 us      | 0.44 us         | 0.12 us      | 1.98 us     |
| 64 KB   | 0.16 us      | 4.84 us         | 0.35 us      | 11.01 us    |
| 1 MB    | 0.32 us      | 417.31 us       | 0.81 us      | 812.76 us   |

The ratio at 1 MB is about 1300x, but the ratio is not the interesting part.
The **shape** is. Across a 32,768x increase in payload, Lockstep goes from
0.09 us to 0.32 us, because the payload is never copied — only a 64-byte
descriptor moves. Cyclone goes from 0.44 us to 417 us, because it serialises
and copies, so its latency tracks the size of the thing being sent.

That is the whole zero-copy argument, and this is it measured against a real
implementation rather than asserted.

Two caveats worth stating plainly. Neither column is a real-time result: on a
stock kernel the tail is the scheduler, for both of them equally. And at 32 B
the gap is only ~5x, because at that size neither transport is doing much —
Lockstep wins the cases it was designed for, and the small-message case is not
one of them.

### Real-time behaviour

Not measured here, and the reason is checked rather than assumed. `rt_validate`
reports on this machine:

```
  PREREQUISITE           STATUS     VALUE
  RLIMIT_RTPRIO          MISSING    0
  PREEMPT_RT kernel      MISSING    5.15.167.4-microsoft-standard-WSL2
  RLIMIT_MEMLOCK         ok         64 MB
  SCHED_FIFO granted     MISSING    refused
  mlockall               ok         yes
```

`RLIMIT_RTPRIO` of 0 means no process here can obtain `SCHED_FIFO` at any
priority, root or not, and a WSL2 kernel cannot bound the tail whatever priority
it is given. So the tool skips and exits 0 rather than printing a figure that
would not mean anything.

On a machine that does qualify, the same binary stops skipping and starts
asserting: zero deadline misses at 1 kHz, no page faults after `mlockall`, and
measured worst-case execution inside the declared deadline. It exits non-zero if
any of those fails. That is the shape the real-time claim would have to take
before this README makes one.

### What is still not measured

No comparison against **iceoryx** or **Fast-DDS**. iceoryx is the one that
matters, since it is also zero-copy and would not show the scaling difference
above; this README's prior-art section says as much and that has not changed.
Fast-DDS is not packaged and would need a source build like Cyclone's.

## On a real robot

The bus runs on a computer; the robot is the body attached to it. There is a
bridge for the **Elegoo Smart Robot Car V4** in [robot/elegoo](robot/elegoo) --
full setup in [robot/elegoo/README.md](robot/elegoo/README.md).

```
[Elegoo car]                          [your computer]
  ESP32  :100  <---- WiFi / TCP ---->  elegoo_bridge
  UNO R3 (motors, ultrasonic)                |
                                    robot/range  robot/drive
                                             |     ^
                                          avoider (control code)
```

Nothing runs on the Arduino or the ESP32; they keep their stock firmware.
Lockstep needs an OS with processes and virtual memory -- which is the entire
reason `offset_ptr` exists -- and a 2 KB ATmega has neither.

The control node is the point. It subscribes to a distance and publishes two
motor speeds. It contains no networking, no JSON, and no knowledge that an
Elegoo car exists, so the same code drives a real car, a simulated one, or a
recorded journal.

**It works with no robot**, which is how the pipeline was verified:

```sh
elegoo_bridge --sim --duration 10000        # synthetic sensors
avoider --duration 8000 --journal drive.jrnl
avoider --replay drive.jrnl                 # twice: same hash both times
```

```
[done] 174 decisions, output hash 5f39c40d1470d372
[replay] 174 decisions from 348 records, hash 5f39c40d1470d372
[replay] 174 decisions from 348 records, hash 5f39c40d1470d372
```

That is not a trivial reproduction. `decide()` deliberately branches on the
clock -- which way it turns depends on the millisecond -- so if replay did not
reproduce the clock, the turns would differ and the hashes would not match.

### What is tested, and what needs the car

`test_elegoo` runs in CI with no hardware: every command encoder pinned to an
exact string, reverse-direction encoding, the motor deadband, rejection of
implausible ultrasonic readings, and the socket layer driven against a fake
robot on loopback including connect, framing, fast failure and mid-session
disconnection.

**Not tested: whether the real firmware agrees with `protocol.hpp`.** That file
was written from Elegoo's published protocol, not from a packet capture, so all
of the uncertainty is concentrated in one `profile` struct and
`elegoo_bridge --probe` prints what your firmware actually replies. If it
disagrees, you edit that struct and nothing else changes.

## Alongside ROS 2

The bridge is the difference between *"rewrite your robot to try this"* and
*"run one more process"*. Your ROS 2 nodes do not change:

```
your ROS 2 nodes  --/imu/data-->  ros_bridge  --ros/imu-->  Lockstep
                  <--/cmd_vel---              <--ros/cmd_vel--
```

```sh
source /opt/ros/jazzy/setup.bash
ros_bridge --in imu:/imu/data --out twist:/cmd_vel --journal run.jrnl
```

Now every IMU message your existing stack publishes is also journalled, and that
journal replays bit for bit. That is the thing `rosbag` cannot do: a bag gives
the data back but not the timing, so the run diverges and the bug you were
chasing does not reappear.

Verified against a live ROS 2 Jazzy graph driven by the stock `ros2 topic pub`
at 20 Hz -- an external ROS node, not this project's code:

```
TOPIC                   PUB PID    STATE   MESSAGES    RATE/s    RING  OVERRUNS
ros/imu                    1202    alive         88      20.0     128         0

[done] imu 138, range 0 bridged in; twist 0 bridged out
```

The 20.0/s there is derived independently, from ring write positions, and agrees
with the rate ROS was publishing at.

### Why each type is hand-written

There is no generic bridge, and the reason is the whole premise of the project.
A ROS message is a generated class holding `std::vector` and `std::string` --
both heap pointers, both refused by `LOCKSTEP_MESSAGE`, because a pointer into
the publisher's heap means nothing to a subscriber that mapped the segment
somewhere else.

So each bridged type gets a flat, padding-free struct and an explicit
conversion. That is a real cost: a new ROS type is a code change, not a config
line. What it buys is a message that crosses a process boundary with no
serialisation and no copy, and hashes reproducibly on replay. A generic bridge
could offer neither.

The conversions are lossy in places, and `test_ros_bridge` pins the losses
rather than only the round trip:

- IMU covariance is dropped, and the rebuilt message reports `-1` (*unknown*)
  rather than leaving `0`, which under REP 145 would falsely claim the
  covariance is known to be exactly zero.
- `Range.frame_id` is dropped, being a variable-length string, and restored from
  a fixed frame on the way out.
- A negative ROS stamp clamps to zero instead of wrapping into a year-2500
  timestamp.

ROS is optional. Without a ROS 2 install on the prefix path, configure prints a
skip line and everything else builds unchanged.

## Looking inside a running bus

```sh
lockstep_top --bus lockstep-robot
```

```
lockstep top  bus topdemo   segment 1.3 MB   watching 4s

TOPIC                   PUB PID    STATE   MESSAGES    RATE/s    RING  OVERRUNS        LAYOUT HASH
robot/range                1145    alive        140      40.0      64         0   a0871b65f0b6f014
robot/status               1145    alive        140      40.0      16         0   46a5240157e012f1
robot/drive                1147    alive        140      40.0      32         0   b955f41b0e19927a

POOL        BLOCK     IN USE       OF  USAGE
0            64 B        112      512  ###.......
1          4.0 KB          2       64  #.........
2         64.0 KB          1       16  #.........
```

Every number here already existed in the segment; none of it cost the publish
path any bookkeeping. Rates are differenced from ring write positions between
refreshes.

It is **strictly an observer** -- it never announces a topic, never publishes,
never takes a slot, never writes a byte. On this bus that matters more than
usual: announcing a topic in order to inspect it would claim the exclusive
publisher slot and lock out the real publisher.

The column worth having is `STATE`. A publisher killed with `SIGKILL` shows up
immediately, before any reaper has run:

```
robot/range                1173     DEAD         72       0.0      64         0
```

which is the screen you stare at when a robot stops responding and you do not
yet know why. `--once` prints a single snapshot for scripts and CI.

## Layout

```
include/lockstep/
  core/offset_ptr.hpp       self-relative pointer, and the rules for using it
  core/relocatable.hpp      is_relocatable / Relocatable / ZeroCopyable
  core/message.hpp          LOCKSTEP_MESSAGE and message_traits
  core/layout_hash.hpp      FNV-1a over the field table, coverage check
  core/padding.hpp          padding at every depth, including inside nested fields
  core/type_name.hpp        compile-time type names to feed the hash
  core/clock.hpp            the one place a raw time source is read
  containers/               inline_vector, inline_string, shm_span
  shm/segment.hpp           shm_open+mmap / CreateFileMapping, offset <-> address
  shm/arena.hpp             slab pools, tagged Treiber free list
  shm/registry.hpp          topic table and the layout-hash handshake
  shm/ring.hpp              the broadcast ring protocol
  shm/bus.hpp               segment header; makes the segment self-describing
  shm/liveness.hpp          dead-publisher detection and repair
  pubsub.hpp                loan / publish / subscribe
  rt/executor.hpp           fixed-priority dispatch, allocation-free
  rt/response_time.hpp      Joseph-Pandya worst-case response time
  replay/journal.hpp        the journal, deterministic clock, output hashing
  net/tcp.hpp               the only socket code; used by the robot bridge
tools/
  lockstep_top.cpp          live view of a running bus; read-only
robot/ros2/
  ros_msgs.hpp              flat, relocatable equivalents of ROS messages
  convert.hpp               ROS <-> Lockstep, pure functions
  ros_bridge.cpp            the bridge node
  test_ros_bridge.cpp       conversion tests; no ROS graph needed
robot/elegoo/
  protocol.hpp              the Elegoo wire format, isolated so it is cheap to
                            correct against real firmware
  robot_msgs.hpp            RobotRange / RobotDrive / RobotStatus
  bridge.cpp                puts a physical car on the bus (--sim works with no car)
  avoider.cpp               obstacle avoidance; records and replays a drive
  test_elegoo.cpp           protocol and socket tests, no robot required
tests/
  test_*.cpp                plain executables, run by CTest
  negative/*.cpp            must fail to compile, CTest asserts WILL_FAIL
  support/                  block.hpp, check.hpp, demo_msgs.hpp
docs/tla/
  Ring.tla                  the ring protocol model
  Ring_spmc.cfg             one publisher: verifies clean
  Ring.cfg                  two publishers: reproduces the torn read, kept
examples/
  01_message_layout.cpp     dumps the field table and layout hash
  demo_node.cpp             one node of the demo, by role
  05_five_node_demo.cpp     five processes, a SIGKILL, and a replay
benchmarks/
  bench_latency.cpp         latency distribution
  rt_validate.cpp           names the missing real-time prerequisite, or asserts
  cyclone/                  the Cyclone DDS baseline, built only if found
verification/relacy/
  ring_seqlock.cpp          the memory ordering, which TLA+ cannot check
```

Tests are plain executables rather than a framework, so the tree builds with
nothing but CMake and a compiler.

## Roadmap

| Phase | Work | Result |
|-------|------|--------|
| 1 | Relocatable types, layout hash, negative tests | **done** — 9 tests, 3 of them compile-failure tests |
| 2 | Shared memory arena, slab pools, topic registry | **done** — two processes map one segment at different bases and share a payload |
| 3 | Ring protocol, TLA+ model check | **done** — TLC found a torn read; one publisher per topic is now enforced |
| 4 | `loan()` / `publish()` / `subscribe()` | **done** — zero copy proved as an address identity |
| 5 | Crash consistency: orphan reclamation, heartbeats | **done** — 10,000 `SIGKILL` injections, 0 corruption |
| 6 | Deadline executor, no post-init alloc | **partly** — 0 allocations enforced by a `new` hook; `SCHED_FIFO` refused here |
| 7 | Journal, replay, bit-exact output hashing | **done** — 1000 replays, 1000 matches |
| 8 | Benchmarks, 5-node demo, ROS 2 interop | **partly** — demo runs and survives a kill; measured against Cyclone DDS (0.32 us vs 417 us at 1 MB); ROS 2 bridge built and verified against a live graph; iceoryx and Fast-DDS not compared |

Real-time numbers will be measured on Linux with `PREEMPT_RT`, isolated cores
and `mlockall`. Windows and WSL2 are for getting the logic right. Timing taken
there wouldn't hold up, so I'm not going to publish any.

## Prior art

[**iceoryx**](https://github.com/eclipse-iceoryx/iceoryx) already does true
zero-copy shared memory pub/sub for robotics and does it well. I'm not trying to
beat it on throughput. What it doesn't give you is deterministic record and
replay, explicit crash-consistency guarantees, or a deadline aware executor with
analytical response time bounds, and those are the reasons this exists. Phase 8
benchmarks against it directly.

`boost::interprocess::offset_ptr` is the reference implementation of the
self-relative pointer idea. Mine differs on copy semantics for the reason given
further up.

ROS 2 interop is a bridge node rather than a full `rmw` implementation, and it
is built -- see [Alongside ROS 2](#alongside-ros-2). That was the right order:
a bridge makes this something you add to a working stack in an afternoon,
whereas an `rmw` backend asks a team to swap out the layer everything else
stands on before they have any reason to trust it. The `rmw` backend would
still be nice eventually; it is still off the critical path.
