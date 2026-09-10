# Model checking the ring protocol

`Ring.tla` models the broadcast ring in `include/lockstep/shm/ring.hpp`. Each
publisher and subscriber action is one shared-memory access, so every
interleaving TLC explores is one the hardware could produce.

## Running it

```sh
curl -sSL -o tla2tools.jar \
  https://github.com/tlaplus/tlaplus/releases/latest/download/tla2tools.jar

java -cp tla2tools.jar tlc2.TLC -config Ring_spmc.cfg Ring.tla   # passes
java -cp tla2tools.jar tlc2.TLC -config Ring.cfg      Ring.tla   # fails, on purpose
```

`scripts/model-check.sh` does both and asserts each gets the outcome it should.

## What it found

The first configuration modelled the ring as multi-producer, because handing
out tickets with a `fetch_add` makes that look free. It is not free. TLC
returned a 16-state counterexample to `NoTornRead`:

| Step | What happens |
|------|--------------|
| 1 | `p1` reserves ticket 0, claims the slot, writes its payload, then stalls |
| 2 | `p2` publishes ticket 1, then ticket 2 — which lands on the same slot as 0 |
| 3 | `p2` claims that slot (`state := 4`) and writes payload 2 |
| 4 | `p1` finally commits ticket 0, storing `state := 1` over the top |

The slot now reads `state == 1` — "ticket 0, committed" — while holding ticket
2's payload. A subscriber at cursor 0 loads 1, copies payload 2, re-loads 1,
sees no change, and accepts. Every check in the protocol passes and the read is
still torn, because a late publisher made the state word go **backwards**, and
the entire seqlock argument rests on it only ever going forwards.

A CAS on the claim does not rescue it. Two publishers can still interleave their
payload writes between one another's claim and commit.

**The stress test in `tests/test_ring.cpp` passed the whole time.** Three
publisher threads, 60,000 messages, a deliberately undersized ring to force
laps — green. That is the argument for model checking in one line: the test
samples whatever interleavings the scheduler happens to pick, and this one is
narrow enough that it never picked it.

## The fix

One publisher per topic, enforced in `registry::announce` — a second publisher
is refused with `attach_status::already_published`.

This costs nothing in practice: a sensor node owns its topic. And phase 7 needs
it regardless, since two publishers racing on one topic produce an interleaving
that cannot be replayed deterministically, which is the property this project
exists to provide.

With one publisher the state word per slot is monotonically non-decreasing,
which is exactly the seqlock precondition. `Ring_spmc.cfg` checks that
configuration exhaustively — one publisher, two subscribers, 65,137 distinct
states, no violation.

## What this does NOT check

**Memory ordering.** Every access in the model is sequentially consistent,
while the implementation uses `acquire`/`release`. So TLC checks that the
*algorithm* is correct, not that the barriers on it are the right ones. A wrong
memory order would sail past this model.

Closing that gap needs a memory-model tool — CDSChecker or relacy for the C++11
model, or ThreadSanitizer against the real implementation. The README lists
those under phase 3; they are **not** done, and nothing in this directory
substitutes for them.
