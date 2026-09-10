# Lockstep

A C++20 message bus for robots, built so that an entire run can be replayed bit
for bit.

You record a run. Later you replay it, and every node re-executes on the same
inputs in the same order and produces byte-identical output. Underneath it all,
the transport is zero-copy shared memory that keeps working when a publisher
gets killed halfway through a write.

Fast pub/sub is a solved problem. The thing you can't get off the shelf is
reproducibility. When a robot does something wrong at 3pm on a Tuesday, you need
to make it do the same wrong thing again on your laptop, and that is what this
is for. Zero copy is in here because it's what makes recording every message
affordable, not because throughput is the point.

> **Where this actually is: phases 1-7 built, phase 8 partly.** 18 tests,
> including 3 that assert code *fails* to compile, all passing on GCC 13.3.
> The type layer, the shared-memory transport, the ring protocol, pub/sub,
> crash consistency, the deadline executor and record/replay are in this repo
> and run. There is a five-node demo you can watch survive a `SIGKILL`.
>
> Two things are **not** done and are not implied to be. The real-time
> *guarantee* needs `PREEMPT_RT`, isolated cores and `CAP_SYS_NICE`, none of
> which this machine has — the executor asks for them, reports that it was
> refused, and marks its own timing as untrustworthy. And the comparison
> against Cyclone DDS, Fast-DDS and iceoryx has never been run, because
> installing them needs root. So there are Lockstep latency numbers below and
> no comparison numbers, and that is deliberate.
>
> **Phase 2 onward is Linux-only.** `shm_open`, `pidfd_open` and robust futexes
> have no Win32 equivalent worth faking. Windows still builds and runs phase 1
> (9 tests); everything above it is skipped there.

## Contents

- [The problem](#the-problem)
- [How it fits together](#how-it-fits-together)
- [Phase 1: the type layer](#phase-1-the-type-layer)
- [What won't compile](#what-wont-compile)
- [Tech stack](#tech-stack)
- [Build and test](#build-and-test)
- [Results](#results)
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
| **Shared memory** | in use, phase 2 | POSIX `shm_open` and `mmap`. Win32 `CreateFileMapping` is still unwritten, so phase 2+ is Linux-only. |
| **Lock free primitives** | in use, phase 3 | `std::atomic` with explicit orders: a tagged Treiber stack for the arena free list, a seqlock per ring slot. |
| **Formal verification** | in use, phase 3 | TLA+ with TLC over the ring protocol. It found a real torn read. CDSChecker, relacy and TSan are still outstanding. |
| **Crash consistency** | in use, phase 5 | `pidfd_open` with a `kill(pid, 0)` fallback, plus a reaper that heals abandoned ring slots. |
| **GCC / Clang** | in use | GCC 13.3 is the primary toolchain now; MSVC still builds phase 1. Clang is still untried. |

### Planned, by phase

Most of what is left is platform API rather than libraries, which is the point.
None of this is in the repo yet.

| Component | Status | Detail |
|-----------|--------|--------|
| **Real time scheduling** | requested, refused here | `mlockall` works. `SCHED_FIFO` needs `CAP_SYS_NICE`, and the guarantee needs `PREEMPT_RT` with `isolcpus`. The executor asks, reports the refusal, and marks its timing untrustworthy. |
| **Benchmarking** | half done | Lockstep's own latency distribution is measured (see Results). Cyclone DDS, Fast-DDS and iceoryx comparisons have never been run: installing them needs root. `scripts/bench-compare.sh` is the starting point and is labelled as never executed. |
| **Memory-model checking** | still planned, phase 3 | CDSChecker or relacy for the C++11 model, ThreadSanitizer against the implementation. The TLA+ model is sequentially consistent, so it does not cover the barriers. |
| **ROS 2 interop** | planned, phase 8 | `rclcpp` for a bridge node. A full `rmw` backend is a stretch goal, not on the critical path. |
| **CI** | planned | GitHub Actions, once there is more than one platform worth keeping green. |

## Build and test

You need CMake 3.20 or newer and a C++20 compiler.

**Linux (everything):**

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
```

```
100% tests passed, 0 tests failed out of 18
```

**Windows (phase 1 only):** the shared-memory transport is POSIX, so the phase 2+
targets are behind `if(UNIX)` and MSVC skips them. `scriptsuild.bat` and
`scripts	est.bat` still work and still give you 9 of 9.

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

### What is not measured

No comparison against Cyclone DDS, Fast-DDS or iceoryx. Installing them needs
root, which was not available. Until someone runs `scripts/bench-compare.sh`,
this README quotes no comparison number.

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
  shm/segment.hpp           shm_open + mmap, offset <-> address translation
  shm/arena.hpp             slab pools, tagged Treiber free list
  shm/registry.hpp          topic table and the layout-hash handshake
  shm/ring.hpp              the broadcast ring protocol
  shm/bus.hpp               segment header; makes the segment self-describing
  shm/liveness.hpp          dead-publisher detection and repair
  pubsub.hpp                loan / publish / subscribe
  rt/executor.hpp           fixed-priority dispatch, allocation-free
  rt/response_time.hpp      Joseph-Pandya worst-case response time
  replay/journal.hpp        the journal, deterministic clock, output hashing
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
| 8 | Benchmarks, 5-node demo | **partly** — demo runs and survives a kill; third-party comparison never run |

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

ROS 2 interop is planned as a bridge node rather than a full `rmw`
implementation. An `rmw` backend would be nice eventually but it is a big
enough job that I'm keeping it off the critical path.
