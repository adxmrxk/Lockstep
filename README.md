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

> **Where this actually is: phase 1 of 8.** The relocatable type layer is
> written and passing, 9 tests including 3 that assert code *fails* to compile.
> The shared memory transport, the executor and the replay engine are designed
> but not built yet. Anything below marked "built" is in this repo and runs.
> Everything else says planned. I'm not quoting benchmark numbers until there's
> something real to benchmark.

## Contents

- [The problem](#the-problem)
- [How it fits together](#how-it-fits-together)
- [Phase 1: the type layer](#phase-1-the-type-layer)
- [What won't compile](#what-wont-compile)
- [Tech stack](#tech-stack)
- [Build and test](#build-and-test)
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
| 2 | Shared memory arena + MPMC ring, model checked | planned |
| 3 | Crash consistency (orphan reclamation, robust futexes) | planned |
| 4 | Deterministic record and replay | planned |
| 5 | Deadline aware executor + response time analysis | planned |

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

Right now the answer is "C++20 and nothing else", and that's on purpose. A
message bus is the thing every other process on the robot links against, so
every dependency I add becomes a dependency for the whole fleet. Phase 1 is
header only with no third party libraries at all.

| Component | What I'm using |
|-----------|----------------|
| **Language** | C++20. Concepts for the type contracts, `std::launder` inside `offset_ptr`, fold expressions for the field checks, constexpr `string_view` for the layout hash. |
| **Build** | CMake 3.20+. Generator agnostic, though I drive it with NMake Makefiles since the VS generator can't see my Build Tools install. |
| **Compiler** | MSVC 19.29 (VS 2019 Build Tools, x64), with `/std:c++20 /Zc:preprocessor /permissive- /W4`. GCC and Clang flags are wired up but I haven't got a modern one on this box yet. |
| **Testing** | CTest driving plain executables. No framework, so the tree builds with just CMake and a compiler. Negative tests use `WILL_FAIL` on the build itself. |
| **Metaprogramming** | A generated 24 arity preprocessor `FOR_EACH`, `__FUNCSIG__` / `__PRETTY_FUNCTION__` scraping for compile time type names, and constexpr FNV-1a for the layout hash. |
| **Dependencies** | None. |

Later phases are where the systems programming shows up, and most of it is
platform API rather than libraries:

| Phase | What it pulls in |
|-------|------------------|
| 2 | POSIX `shm_open` + `mmap`, Win32 `CreateFileMapping` for the dev path |
| 3 | `std::atomic` with explicit memory orders, `atomic_ref`, futex wait/wake |
| 3 | TLA+ / TLC for the protocol, CDSChecker or relacy for the C++ memory model, ThreadSanitizer for the implementation |
| 5 | Robust futexes (`PTHREAD_MUTEX_ROBUST`), `pidfd_open` for liveness detection |
| 6 | Linux `PREEMPT_RT`, `SCHED_DEADLINE` / `SCHED_FIFO`, `isolcpus`, `nohz_full`, `mlockall` |
| 8 | Cyclone DDS, Fast-DDS and iceoryx to benchmark against, HdrHistogram for the latency numbers |
| 8 | `rclcpp` for the ROS 2 bridge node |
| any | GitHub Actions once there is more than one platform to keep green |

## Build and test

You need CMake 3.20 or newer and a C++20 compiler. I develop against MSVC 19.29
(Visual Studio 2019 Build Tools, x64).

```bat
scripts\build.bat            :: configure and build
scripts\test.bat             :: build, then ctest including the negative tests
```

Or by hand, from a developer command prompt:

```bat
cmake -S . -B build -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
```

CMake sets `/Zc:preprocessor` and it is not optional. `LOCKSTEP_MESSAGE` needs
conformant `__VA_ARGS__` expansion and the legacy MSVC preprocessor breaks it.

```
100% tests passed, 0 tests failed out of 9

    test_offset_ptr ...................   Passed
    test_relocation ...................   Passed
    test_containers ...................   Passed
    test_message ......................   Passed
    test_layout_hash ..................   Passed
    test_provenance ...................   Passed
    compile_fail.raw_pointer_field ....   Passed
    compile_fail.incomplete_field_list    Passed
    compile_fail.std_string_field .....   Passed
```

## Layout

```
include/lockstep/
  core/offset_ptr.hpp       self-relative pointer, and the rules for using it
  core/relocatable.hpp      is_relocatable / Relocatable / ZeroCopyable
  core/message.hpp          LOCKSTEP_MESSAGE and message_traits
  core/layout_hash.hpp      FNV-1a over the field table, coverage and padding checks
  core/type_name.hpp        compile-time type names to feed the hash
  containers/               inline_vector, inline_string, shm_span
tests/
  test_*.cpp                plain executables, run by CTest
  negative/*.cpp            must fail to compile, CTest asserts WILL_FAIL
  support/block.hpp         aligned allocation standing in for a mapped segment
examples/
  01_message_layout.cpp     dumps the field table and layout hash
```

Tests are plain executables rather than a framework, so the tree builds with
nothing but CMake and a compiler.

## Roadmap

| Phase | Work | What it produces |
|-------|------|------------------|
| 1 | Relocatable types, layout hash, negative tests | **done** |
| 2 | Shared memory arena, slab pools, topic registry | a segment two processes can map |
| 3 | MPMC ring protocol, TLA+ / CDSChecker model check | a published protocol spec |
| 4 | `loan()` / `publish()` / `subscribe()` | real zero copy, no memcpy |
| 5 | Crash consistency: orphan reclamation, heartbeats | 10k `SIGKILL` injections, no corruption |
| 6 | Deadline executor: `SCHED_DEADLINE`, no post-init alloc | no deadline misses under load |
| 7 | Journal, replay, bit-exact output hashing | 1000 replays, 100% hash match |
| 8 | Benchmarks vs Cyclone / Fast-DDS / iceoryx, 5-node demo | latency histograms, crash and replay demo |

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
