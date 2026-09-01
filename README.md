# Lockstep

**A robot message bus where the entire run is bit-exactly reproducible.**

Record a robot's execution, replay it offline, and every node re-executes with
identical inputs in identical order and produces byte-identical output — while
the bus underneath is zero-copy, crash-tolerant, and deadline-scheduled.

Fast pub/sub already exists. What no open-source middleware gives you is the
thing every serious autonomy company builds internally: when the robot does
something wrong at 3pm on Tuesday, you need to make it do the exact same wrong
thing again on your laptop. Determinism is the goal here; zero copy is the
mechanism that makes it affordable.

> **Status: phase 1 of 8 complete.** The relocatable type layer is built,
> tested, and passing — 9/9 tests including 3 negative compile tests. The
> shared-memory transport, executor, and replay engine are designed but not yet
> written. Everything claimed below as *built* is in this repository and runs;
> everything else is explicitly marked planned. No benchmark numbers are quoted
> until there is something to benchmark.

---

## Table of Contents

- [The Problem](#the-problem)
- [Design](#design)
- [Phase 1: The Relocatable Type Layer](#phase-1-the-relocatable-type-layer)
- [What the Type System Rejects](#what-the-type-system-rejects)
- [A Provenance Miscompile, and the Fix](#a-provenance-miscompile-and-the-fix)
- [Building and Testing](#building-and-testing)
- [Project Structure](#project-structure)
- [Roadmap](#roadmap)
- [Prior Art](#prior-art)

---

## The Problem

A perception-to-control stack passes 24 MB camera frames between processes at
30 Hz. Three things go wrong with the usual answers:

1. **DDS copies.** Fast-DDS and Cyclone serialize the message, copy it into a
   transport buffer, and copy it out again. At 24 MB the copies alone dominate
   the latency budget and blow the tail.
2. **Nothing is reproducible.** ROS 2 bag replay reproduces the *messages*, not
   the *execution*: callback interleaving, wall-clock reads, and thread
   scheduling all differ on replay, so the bug you recorded frequently refuses
   to reappear.
3. **A crash is unbounded.** Kill a publisher mid-write and the shared state it
   was touching is left in whatever condition it was in. Subscribers block, or
   read a torn message, or the segment needs a full teardown.

Lockstep attacks all three from the same substrate: a shared-memory segment
where messages are written once in their final location, every dispatch decision
is journaled, and every slot carries the ownership metadata needed to reclaim it
from a dead process.

## Design

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

The five pillars, in dependency order:

| # | Pillar | Status |
|---|--------|--------|
| 1 | Relocatable zero-copy types | **built** |
| 2 | Shared-memory arena + MPMC ring, model-checked | planned |
| 3 | Crash consistency (orphan reclamation, robust futexes) | planned |
| 4 | Deterministic record & replay | planned |
| 5 | Deadline-aware executor + response-time analysis | planned |

## Phase 1: The Relocatable Type Layer

A shared-memory segment maps at a **different base address in every process**.
A `std::string`, a `std::vector`, or any raw pointer written by the publisher is
therefore meaningless to the subscriber — it encodes an address in an address
space the reader does not have. The entire type layer exists to make that class
of bug a compile error.

### `ls::offset_ptr<T>` — a pointer that survives relocation

Stores a signed byte offset from its own address rather than an absolute
address. When the enclosing block is copied elsewhere, the pointer and its
target move by the same delta, so the offset stays correct and **the raw bytes
never have to be rewritten**:

```cpp
std::memcpy(dst_block, src_block, block_size);   // every internal link intact
```

Unlike `boost::interprocess::offset_ptr`, copy and assignment are **defaulted**,
so `offset_ptr` is trivially copyable and the bus can `static_assert` that a
whole message is memcpy-able and treat it as bytes. The tradeoff is explicit:
lifting a single `offset_ptr` out of its block is a bug, and `.get()` is the
supported way to leave.

### `LOCKSTEP_MESSAGE` — opt-in with proof obligations

C++20 has no reflection, so a struct cannot be inspected for hidden pointers.
Lockstep inverts the default instead: **class types are not relocatable unless
declared**, and the declaration checks every member.

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

That single line static_asserts that every member is relocatable, that the type
is standard-layout, trivially copyable and trivially destructible, and that the
field list **accounts for every byte of the struct** — then computes a 64-bit
layout hash the two peers exchange at connect time, so a stale node cannot
misread a struct that was reordered under it.

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

`padding yes` is not cosmetic. Interior padding bytes are indeterminate, so a
message that has any **must be zero-filled before publish** or a replay hash
taken over the slot will not reproduce. The traits report it so the publisher
can enforce it.

### Containers

| Type | Storage | Use for |
|------|---------|---------|
| `inline_vector<T, N>` | inline, fixed capacity | bounded lists in a message |
| `inline_string<N>` | inline, fixed capacity | frame ids, sensor names |
| `shm_span<T>` | `offset_ptr` + length | large payloads in the arena |

A 24 MB frame has no business inside a fixed-size message slot, so
`CameraFrame` stays **64 bytes** and the pixels live in the arena with a span
pointing at them.

All three zero their unused capacity rather than merely resetting a length.
That is a determinism requirement, not tidiness: a recycled slot that leaks the
previous message's bytes would change a replay hash taken over the whole slot.

## What the Type System Rejects

Three negative compile tests assert that the following **fail to build**. CTest
runs them with `WILL_FAIL`, so a regression that lets them through is a test
failure.

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

A `std::string` member trips three assertions at once: not relocatable, not
trivially copyable, not trivially destructible.

## A Provenance Miscompile, and the Fix

The first optimized build produced wrong code. On MSVC 19.29 at `/O2`:

```cpp
int x = 42;
offset_ptr<int> p = &x;
*p = 43;
assert(x == 43);        // FAILED at /O2, passed at /Od
```

**Cause.** Reconstructing a pointer from a stored offset is pointer arithmetic
whose base is a known object — the `offset_ptr` itself, 8 bytes wide. The
compiler is entitled to assume such arithmetic stays inside that object, and
therefore that a store through the result cannot alias anything else. So the
read of `x` was served from a cached 42.

The obvious diagnosis — escape analysis deciding `x` never escapes — is wrong,
and testing it is what made that clear. None of these fixed it:

| Attempt | Result |
|---|---|
| `char*` arithmetic instead of an `intptr_t` round trip | still broken |
| `__declspec(noinline)` encode, so the address reaches an opaque callee | still broken |
| Storing the target address through a `volatile` file-scope pointer | still broken |
| `_ReadWriteBarrier()`, `std::atomic_signal_fence(acq_rel)` | still broken |

Passing the address to an opaque callee not fixing it is what ruled escape
analysis out: the target *did* escape, and the store was still dropped.

**Fix.** Launder the *base* of the arithmetic so the compiler stops attributing
the result to the base object. Three constructs work — `std::launder`, a
`noinline` identity on the base, and a `volatile` round trip on the base — and
`std::launder` is the only one that costs nothing: it emits no instruction and
only blocks the provenance inference.

```cpp
template <class T>
inline T* launder_offset(void* base, std::int64_t offset) noexcept {
  return std::launder(reinterpret_cast<T*>(static_cast<char*>(base) + offset));
}
```

`tests/test_provenance.cpp` pins this down with the cross-object cases that
miscompiled. Reverting `launder_offset` to the naive form fails that test at
`/O2` — verified, so the regression test is known to actually catch the bug
rather than passing vacuously.

Separately, and for a different reason, **an `offset_ptr` and its target must
live in the same relocatable block**: relocation copies a block's bytes
elsewhere, so an offset reaching outside the block lands on whatever happens to
sit at that address in the destination. That is a semantic invariant the arena
enforces, not a codegen hazard, and it survives the fix above.

## Building and Testing

Requires CMake 3.20+ and a C++20 compiler. Developed against MSVC 19.29
(Visual Studio 2019 Build Tools, x64).

```bat
scripts\build.bat            :: configure + build
scripts\test.bat             :: build + ctest, including negative compile tests
```

Or directly, from a developer command prompt:

```bat
cmake -S . -B build -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
```

`/Zc:preprocessor` is required and set by CMake: `LOCKSTEP_MESSAGE` relies on
conformant `__VA_ARGS__` expansion, which the legacy MSVC preprocessor breaks.

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

## Project Structure

```
include/lockstep/
  core/offset_ptr.hpp       self-relative pointer + the relocation invariant
  core/relocatable.hpp      is_relocatable / Relocatable / ZeroCopyable
  core/message.hpp          LOCKSTEP_MESSAGE and message_traits
  core/layout_hash.hpp      FNV-1a over the field table, coverage + padding checks
  core/type_name.hpp        compile-time type names for the hash
  containers/               inline_vector, inline_string, shm_span
tests/
  test_*.cpp                positive tests, plain executables driven by CTest
  negative/*.cpp            must fail to compile; CTest asserts WILL_FAIL
  support/block.hpp         aligned allocation standing in for a mapped segment
examples/
  01_message_layout.cpp     dumps the field table and layout hash
```

Tests are plain executables rather than a framework, so the tree configures and
builds with nothing but CMake and a compiler.

## Roadmap

| Phase | Work | Deliverable |
|-------|------|-------------|
| 1 | Relocatable types, layout hash, negative tests | **done** |
| 2 | Shared-memory arena, slab pools, topic registry | segment two processes can map |
| 3 | MPMC ring protocol + TLA+ / CDSChecker model check | published protocol spec |
| 4 | `loan()` / `publish()` / `subscribe()` | true zero copy, no memcpy |
| 5 | Crash consistency: orphan reclamation, heartbeats | 10k `SIGKILL` injections, zero corruptions |
| 6 | Deadline executor: `SCHED_DEADLINE`, zero post-init alloc | zero deadline misses under load |
| 7 | Journal + replay + bit-exact output hashing | 1000 replays, 100% hash match |
| 8 | Benchmarks vs Cyclone / Fast-DDS / iceoryx; 5-node demo | latency histograms, crash + replay demo |

Real-time numbers will be measured on Linux with `PREEMPT_RT`, isolated cores
and `mlockall`. Windows and WSL2 are for functional development only; timing
figures taken there would not be defensible and none will be published.

## Prior Art

[**iceoryx**](https://github.com/eclipse-iceoryx/iceoryx) already does true
zero-copy shared-memory pub/sub for robotics, and does it well. Lockstep is not
trying to beat it at throughput. The differentiators are deterministic record
and replay, explicit crash-consistency guarantees, and a deadline-aware executor
with analytical response-time bounds — none of which iceoryx provides. Phase 8
benchmarks against it directly.

`boost::interprocess::offset_ptr` is the reference implementation of the
self-relative pointer idea; Lockstep's differs deliberately in copy semantics,
for the reason given above.

ROS 2 interop is planned as a bridge node, not a full `rmw` implementation. An
`rmw` backend is a stretch goal and is deliberately not on the critical path.
