// Phase 5's acceptance test: kill publishers with SIGKILL, repeatedly, and
// assert the bus is never left corrupt or wedged.
//
// Each round forks a real publisher process, lets it run for a random slice of
// time, SIGKILLs it, then checks four things:
//
//   1. no subscriber ever accepts a torn message -- every field of an accepted
//      ImuSample is derived from one counter, so incoherence is detectable;
//   2. the registry survives: same topic, same layout hash, still exactly one
//      entry;
//   3. the dead publisher's exclusive slot is reclaimed, so a replacement can
//      take the topic over -- which the next round proves by doing it;
//   4. a ring slot left mid-write is healed and then SKIPPED by readers, not
//      handed over as if it were a real message.
//
// Iterations default low so the suite stays fast. Set LOCKSTEP_CRASH_ITERS to
// run the 10,000 the roadmap asks for.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include "lockstep/core/process.hpp"
#include "lockstep/pubsub.hpp"
#include "lockstep/shm/liveness.hpp"
#include "support/check.hpp"
#include "support/process.hpp"
#include "support/demo_msgs.hpp"

#ifndef LOCKSTEP_CRASH_CHILD_BINARY
#error "LOCKSTEP_CRASH_CHILD_BINARY must be defined by the build"
#endif

namespace {

// std::getenv is perfectly correct here and MSVC's C4996 objection is about
// thread-safety in a program that mutates the environment. This one does not.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
inline const char* read_env(const char* name) { return std::getenv(name); }
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

bool coherent(const ImuSample& s) {
  const float f = static_cast<float>(s.stamp_ns & 0xffffu);
  return s.ax == f && s.ay == f && s.az == f && s.gx == f && s.gy == f && s.gz == f;
}

ls::test::child_process spawn_child(const std::string& segment, unsigned hold_us) {
  return ls::test::spawn(LOCKSTEP_CRASH_CHILD_BINARY,
                         {segment, std::to_string(hold_us)});
}

struct totals {
  std::uint64_t rounds = 0;
  std::uint64_t killed = 0;
  std::uint64_t reclaimed = 0;
  std::uint64_t had_publisher = 0;  // rounds where the child got as far as
                                    // claiming the topic before we killed it
  std::uint64_t healed = 0;
  std::uint64_t accepted = 0;
  std::uint64_t abandoned = 0;
  std::uint64_t overruns = 0;
  std::uint64_t torn = 0;
};

void sigkill_storm() {
  const char* iters_env = read_env("LOCKSTEP_CRASH_ITERS");
  const int iters = iters_env ? std::atoi(iters_env) : 250;

  const std::string name = "lockstep-crash-" + std::to_string(ls::self_pid());
  ls::segment::unlink(name);
  ls::bus b = ls::bus::create(name, {{64, 512}, {4096, 32}, {65536, 8}});

  ls::reaper rp(b);
  totals t;

  std::mt19937 rng(12345);
  std::uniform_int_distribution<int> delay_us(200, 3000);
  std::uniform_int_distribution<int> hold_us(0, 120);

  for (int round = 0; round < iters; ++round) {
    ++t.rounds;

    ls::test::child_process child =
        spawn_child(name, static_cast<unsigned>(hold_us(rng)));
    LS_CHECK(child.valid());
    if (!child.valid()) break;

    ls::test::sleep_us(static_cast<unsigned>(delay_us(rng)));

    // The harshest kill each platform has: no handler, no unwinding, no chance
    // to clean up. This is the failure mode the phase exists for.
    ls::test::kill_hard(child);
    ls::test::wait_for(child);
    ++t.killed;

    // Sample AFTER the kill, not before. Not every child survives long enough
    // to claim the topic -- with kill delays down at 200us some die inside
    // publisher::create -- and reading this beforehand races the child, which
    // can finish claiming in the gap between the read and the signal. Once it
    // is dead the value is frozen and the comparison against reclaimed is
    // exact.
    {
      ls::topic_entry* pre = b.topics().find("imu/crash");
      if (pre != nullptr &&
          pre->publisher_pid.load(std::memory_order_acquire) == child.pid) {
        ++t.had_publisher;
      }
    }

    // Repair.
    const ls::reap_report r = rp.scan();
    t.reclaimed += r.publishers_reclaimed;
    t.healed += r.slots_healed;

    // Drain whatever the dead publisher managed to publish, and check that
    // nothing incoherent is ever handed over as a valid message.
    ls::topic_entry* e = b.topics().find("imu/crash");
    if (e != nullptr && e->ring_offset.load(std::memory_order_acquire) != 0) {
      try {
        auto sub = ls::subscriber<ImuSample>::attach(b, "imu/crash");
        for (int i = 0; i < 4096; ++i) {
          ls::sample<ImuSample> s;
          const auto rr = sub.take(s);
          if (rr == ls::read_result::empty) break;
          if (rr == ls::read_result::ok) {
            ImuSample copy{};
            if (s.copy_out(copy)) {
              ++t.accepted;
              if (!coherent(copy)) ++t.torn;
            }
          } else if (rr == ls::read_result::abandoned) {
            ++t.abandoned;
          } else if (rr == ls::read_result::overrun) {
            ++t.overruns;
          }
        }
      } catch (const std::exception& ex) {
        std::fprintf(stderr, "  attach failed in round %d: %s\n", round, ex.what());
        LS_CHECK(false);
      }

      // The registry must be intact after every kill.
      LS_CHECK_EQ(b.topics().used(), 1u);
      LS_CHECK_EQ(e->layout_hash, ls::message_traits<ImuSample>::layout_hash);
      LS_CHECK_EQ(e->message_size, sizeof(ImuSample));

      // And the dead publisher must no longer hold the topic, or the next round
      // could not take it over.
      LS_CHECK_EQ(e->publisher_pid.load(std::memory_order_acquire), 0u);
    }
  }

  std::fprintf(stderr,
               "  [crash] rounds=%llu killed=%llu had_publisher=%llu\n"
               "  [crash] reclaimed=%llu healed=%llu\n"
               "  [crash] accepted=%llu abandoned=%llu overruns=%llu torn=%llu\n",
               (unsigned long long)t.rounds,
               (unsigned long long)t.killed,
               (unsigned long long)t.had_publisher,
               (unsigned long long)t.reclaimed,
               (unsigned long long)t.healed,
               (unsigned long long)t.accepted,
               (unsigned long long)t.abandoned,
               (unsigned long long)t.overruns,
               (unsigned long long)t.torn);

  // THE property.
  LS_CHECK_EQ(t.torn, 0u);

  // Guard against the test passing vacuously: the publishers must actually have
  // published, and the reaper must actually have had work to do.
  LS_CHECK(t.accepted > 0);
  LS_CHECK(t.healed > 0);  // kills really did land between claim and commit
  // Every publisher that got as far as owning the topic was reclaimed. Not
  // every kill, because a child killed inside publisher::create never owned it.
  LS_CHECK_EQ(t.reclaimed, t.had_publisher);
}

// A publisher that dies holding the topic must not lock it out forever.
void a_dead_publishers_topic_can_be_taken_over() {
  const std::string name = "lockstep-takeover-" + std::to_string(ls::self_pid());
  ls::segment::unlink(name);
  ls::bus b = ls::bus::create(name, {{64, 256}, {4096, 8}, {65536, 4}});

  ls::test::child_process child = spawn_child(name, 0);
  LS_CHECK(child.valid());
  ls::test::sleep_us(20000);

  ls::topic_entry* e = b.topics().find("imu/crash");
  LS_CHECK(e != nullptr);
  const std::uint32_t dead_pid = e->publisher_pid.load();
  LS_CHECK_EQ(dead_pid, child.pid);

  // Before the kill, this process cannot publish on that topic.
  ls::topic_entry* blocked = nullptr;
  LS_CHECK(b.topics().announce<ImuSample>("imu/crash", &blocked, true, 999999) ==
           ls::attach_status::already_published);

  ls::test::kill_hard(child);
  ls::test::wait_for(child);

  // The pid is gone, so the reaper releases the topic.
  LS_CHECK(!ls::process_alive(dead_pid));
  ls::reaper rp(b);
  const auto r = rp.scan();
  LS_CHECK_EQ(r.publishers_reclaimed, 1u);

  // And now a replacement can take it over and keep publishing.
  auto pub = ls::publisher<ImuSample>::create(b, "imu/crash", 64);
  auto ln = pub.loan_message();
  ln->stamp_ns = 7;
  const float f = 7.0f;
  ln->ax = f; ln->ay = f; ln->az = f; ln->gx = f; ln->gy = f; ln->gz = f;
  LS_CHECK(pub.publish(ln));

  auto sub = ls::subscriber<ImuSample>::attach(b, "imu/crash");
  ls::sample<ImuSample> s;
  bool saw_ours = false;
  for (int i = 0; i < 256; ++i) {
    const auto rr = sub.take(s);
    if (rr == ls::read_result::empty) break;
    if (rr == ls::read_result::ok && s->stamp_ns == 7) saw_ours = true;
  }
  LS_CHECK(saw_ours);
}

// A slot left mid-write must be skipped, never served as a real message.
void an_abandoned_slot_is_skipped_not_served() {
  const std::string name = "lockstep-abandon-" + std::to_string(ls::self_pid());
  ls::segment::unlink(name);
  ls::bus b = ls::bus::create(name, {{64, 256}, {4096, 8}, {65536, 4}});

  auto pub = ls::publisher<ImuSample>::create(b, "imu/x", 8);

  auto good = pub.loan_message();
  good->stamp_ns = 1;
  good->ax = 1.0f; good->ay = 1.0f; good->az = 1.0f;
  good->gx = 1.0f; good->gy = 1.0f; good->gz = 1.0f;
  pub.publish(good);

  // Claim a slot and never commit it: exactly what a SIGKILL between loan and
  // publish leaves behind.
  auto stranded = pub.loan_message();
  stranded->stamp_ns = 999;
  (void)stranded;

  auto sub = ls::subscriber<ImuSample>::attach(b, "imu/x");
  ls::sample<ImuSample> s;
  LS_CHECK(sub.take(s) == ls::read_result::ok);
  LS_CHECK_EQ(s->stamp_ns, 1u);

  // The stranded ticket blocks the reader: it is claimed, so never committed.
  LS_CHECK(sub.take(s) == ls::read_result::empty);

  // Healing it unblocks the reader without serving the half-written bytes.
  ls::ring rg = pub.get_ring();
  LS_CHECK_EQ(ls::reaper::heal(rg), 1u);
  LS_CHECK(sub.take(s) == ls::read_result::abandoned);
  LS_CHECK(sub.take(s) == ls::read_result::empty);
}

}  // namespace

int main() {
  an_abandoned_slot_is_skipped_not_served();
  a_dead_publishers_topic_can_be_taken_over();
  sigkill_storm();
  return ls::test::summary("crash_consistency");
}
