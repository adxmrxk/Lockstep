------------------------------- MODULE Ring -------------------------------
(***************************************************************************)
(* A TLA+ model of the broadcast ring protocol in                          *)
(* include/lockstep/shm/ring.hpp.                                          *)
(*                                                                         *)
(* The model exists to answer one question that testing cannot settle:     *)
(* is there ANY interleaving of publishers and subscribers in which a      *)
(* subscriber accepts a message assembled from two different publications? *)
(* A stress test can only tell you it did not happen on the interleavings  *)
(* the scheduler happened to produce. TLC enumerates all of them.          *)
(*                                                                         *)
(* Each publisher and subscriber step below is one shared-memory access,   *)
(* so any interleaving TLC explores is one the hardware could produce.     *)
(* The publisher's three writes are modelled as three separate steps       *)
(* precisely so a subscriber can be scheduled between any two of them.     *)
(*                                                                         *)
(* Modelled:      the seqlock state word, slot reuse (laps), multiple      *)
(*                publishers taking disjoint tickets, torn-read detection. *)
(* Not modelled:  memory reordering. Every access here is sequentially     *)
(*                consistent, whereas the implementation uses acquire and  *)
(*                release. So this checks the ALGORITHM, not the barriers. *)
(*                Checking the barriers needs a memory-model tool -- see   *)
(*                the note in docs/tla/README.md.                          *)
(***************************************************************************)
EXTENDS Naturals

CONSTANTS Capacity,    \* ring slots; a power of two in the implementation
          MaxTicket,   \* bound on publications, to keep the state space finite
          Pubs,        \* set of publisher identities
          Subs         \* set of subscriber identities

ASSUME Capacity \in Nat /\ Capacity > 0
ASSUME MaxTicket \in Nat

VARIABLES
  state,     \* slot -> the atomic state word: 2t while writing, 2t+1 committed
  data,      \* slot -> the ticket whose payload currently occupies the slot
  writePos,  \* next ticket to hand out
  ppc,       \* publisher -> program counter
  pt,        \* publisher -> the ticket it is currently publishing
  spc,       \* subscriber -> program counter
  cursor,    \* subscriber -> its own read position
  sSeen,     \* subscriber -> the state word it loaded in step 1
  sData,     \* subscriber -> the payload it copied in step 3
  torn       \* set once a subscriber accepts a payload that is not its own

vars == <<state, data, writePos, ppc, pt, spc, cursor, sSeen, sData, torn>>

Slots     == 0 .. (Capacity - 1)
SlotOf(t) == t % Capacity

(* The oldest ticket still resident in the ring, mirroring the C++. *)
Oldest == IF writePos > Capacity THEN writePos - Capacity ELSE 0

Init ==
  /\ state    = [i \in Slots |-> 0]
  /\ data     = [i \in Slots |-> 0]
  /\ writePos = 0
  /\ ppc      = [p \in Pubs |-> "reserve"]
  /\ pt       = [p \in Pubs |-> 0]
  /\ spc      = [s \in Subs |-> "load1"]
  /\ cursor   = [s \in Subs |-> 0]
  /\ sSeen    = [s \in Subs |-> 0]
  /\ sData    = [s \in Subs |-> 0]
  /\ torn     = FALSE

(***************************************************************************)
(* Publisher: reserve a ticket, mark the slot busy, write the payload,     *)
(* then commit. Four separate steps, so a subscriber may run between any.  *)
(***************************************************************************)

Reserve(p) ==
  /\ ppc[p] = "reserve"
  /\ writePos < MaxTicket
  /\ pt'       = [pt  EXCEPT ![p] = writePos]
  /\ writePos' = writePos + 1
  /\ ppc'      = [ppc EXCEPT ![p] = "claim"]
  /\ UNCHANGED <<state, data, spc, cursor, sSeen, sData, torn>>

Claim(p) ==
  /\ ppc[p] = "claim"
  /\ state' = [state EXCEPT ![SlotOf(pt[p])] = 2 * pt[p]]
  /\ ppc'   = [ppc   EXCEPT ![p] = "write"]
  /\ UNCHANGED <<data, writePos, pt, spc, cursor, sSeen, sData, torn>>

WritePayload(p) ==
  /\ ppc[p] = "write"
  /\ data' = [data EXCEPT ![SlotOf(pt[p])] = pt[p]]
  /\ ppc'  = [ppc  EXCEPT ![p] = "commit"]
  /\ UNCHANGED <<state, writePos, pt, spc, cursor, sSeen, sData, torn>>

Commit(p) ==
  /\ ppc[p] = "commit"
  /\ state' = [state EXCEPT ![SlotOf(pt[p])] = 2 * pt[p] + 1]
  /\ ppc'   = [ppc   EXCEPT ![p] = "reserve"]
  /\ UNCHANGED <<data, writePos, pt, spc, cursor, sSeen, sData, torn>>

(***************************************************************************)
(* Subscriber: load the state word, decide, copy, then re-load the state   *)
(* word and accept only if it did not move.                                *)
(***************************************************************************)

Load1(s) ==
  /\ spc[s] = "load1"
  /\ sSeen' = [sSeen EXCEPT ![s] = state[SlotOf(cursor[s])]]
  /\ spc'   = [spc   EXCEPT ![s] = "check"]
  /\ UNCHANGED <<state, data, writePos, ppc, pt, cursor, sData, torn>>

Check(s) ==
  /\ spc[s] = "check"
  /\ LET want == 2 * cursor[s] + 1 IN
       IF sSeen[s] < want
         THEN /\ spc'    = [spc EXCEPT ![s] = "load1"]      \* empty, wait
              /\ cursor' = cursor
         ELSE IF sSeen[s] > want
           THEN /\ cursor' = [cursor EXCEPT ![s] =           \* lapped, resync
                               IF Oldest > cursor[s] THEN Oldest ELSE cursor[s] + 1]
                /\ spc'    = [spc EXCEPT ![s] = "load1"]
           ELSE /\ spc'    = [spc EXCEPT ![s] = "copy"]      \* ours, read it
                /\ cursor' = cursor
  /\ UNCHANGED <<state, data, writePos, ppc, pt, sSeen, sData, torn>>

Copy(s) ==
  /\ spc[s] = "copy"
  /\ sData' = [sData EXCEPT ![s] = data[SlotOf(cursor[s])]]
  /\ spc'   = [spc   EXCEPT ![s] = "load2"]
  /\ UNCHANGED <<state, data, writePos, ppc, pt, cursor, sSeen, torn>>

Load2(s) ==
  /\ spc[s] = "load2"
  /\ IF state[SlotOf(cursor[s])] = sSeen[s]
       THEN \* accepted: the payload must be the one this cursor asked for
            /\ torn'   = (torn \/ (sData[s] # cursor[s]))
            /\ cursor' = [cursor EXCEPT ![s] = cursor[s] + 1]
       ELSE \* the word moved, so the copy was torn: discard and retry
            /\ torn'   = torn
            /\ cursor' = cursor
  /\ spc' = [spc EXCEPT ![s] = "load1"]
  /\ UNCHANGED <<state, data, writePos, ppc, pt, sSeen, sData>>

Next ==
  \/ \E p \in Pubs : Reserve(p) \/ Claim(p) \/ WritePayload(p) \/ Commit(p)
  \/ \E s \in Subs : Load1(s) \/ Check(s) \/ Copy(s) \/ Load2(s)

Spec == Init /\ [][Next]_vars

(***************************************************************************)
(* THE PROPERTY. A subscriber never accepts a payload belonging to a       *)
(* ticket other than the one its cursor was reading.                       *)
(***************************************************************************)
NoTornRead == torn = FALSE

(* Sanity bound so a runaway cursor shows up as an invariant violation      *)
(* rather than as an unbounded state space.                                 *)
CursorBounded == \A s \in Subs : cursor[s] <= MaxTicket + 1

=============================================================================
