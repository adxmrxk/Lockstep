// The topic table, and the layout-hash handshake that is the whole reason
// phase 1 exists.
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "lockstep/shm/registry.hpp"
#include "lockstep/shm/segment.hpp"
#include "support/check.hpp"
#include "support/demo_msgs.hpp"

namespace {

std::string unique_name(const char* tag) {
  return std::string("lockstep-reg-") + tag + "-" + std::to_string(::getpid());
}

struct fixture {
  ls::segment seg;
  ls::registry reg;

  explicit fixture(const char* tag) {
    const std::string name = unique_name(tag);
    ls::segment::unlink(name);
    seg = ls::segment::create(name, 4096 + ls::registry::bytes_required());
    reg = ls::registry::construct(seg.base(), 4096);
  }
};

void announce_then_find() {
  fixture f("announce");
  ls::topic_entry* e = nullptr;

  LS_CHECK(f.reg.announce<ImuSample>("imu/raw", &e) == ls::attach_status::ok);
  LS_CHECK(e != nullptr);
  LS_CHECK(e->topic_name() == std::string_view("imu/raw"));
  LS_CHECK_EQ(e->layout_hash, ls::message_traits<ImuSample>::layout_hash);
  LS_CHECK_EQ(e->message_size, sizeof(ImuSample));
  LS_CHECK_EQ(f.reg.used(), 1u);

  LS_CHECK(f.reg.find("imu/raw") == e);
  LS_CHECK(f.reg.find("imu/cooked") == nullptr);
}

void attaching_with_the_right_type_succeeds() {
  fixture f("attach_ok");
  ls::topic_entry* pub = nullptr;
  ls::topic_entry* sub = nullptr;

  LS_CHECK(f.reg.announce<CameraFrame>("cam/front", &pub) == ls::attach_status::ok);
  LS_CHECK(f.reg.attach<CameraFrame>("cam/front", &sub) == ls::attach_status::ok);
  LS_CHECK_EQ(pub, sub);
}

// The case the hash is for: same size, same field names, two fields swapped.
// A size check alone would wave this through.
void attaching_with_a_reordered_struct_is_refused() {
  fixture f("attach_bad");
  ls::topic_entry* pub = nullptr;
  ls::topic_entry* sub = nullptr;

  LS_CHECK(f.reg.announce<CameraFrame>("cam/front", &pub) == ls::attach_status::ok);

  // Precondition: the two types really are the same size, so this test is
  // exercising the hash and not accidentally passing on a size difference.
  LS_CHECK_EQ(sizeof(CameraFrame), sizeof(CameraFrameReordered));
  LS_CHECK(ls::message_traits<CameraFrame>::layout_hash !=
           ls::message_traits<CameraFrameReordered>::layout_hash);

  LS_CHECK(f.reg.attach<CameraFrameReordered>("cam/front", &sub) ==
           ls::attach_status::layout_mismatch);
}

void attaching_to_a_missing_topic_is_refused() {
  fixture f("missing");
  ls::topic_entry* e = nullptr;
  LS_CHECK(f.reg.attach<ImuSample>("nope", &e) == ls::attach_status::not_found);
}

void announcing_the_same_topic_twice_reuses_the_entry() {
  fixture f("twice");
  ls::topic_entry* a = nullptr;
  ls::topic_entry* b = nullptr;

  LS_CHECK(f.reg.announce<ImuSample>("imu", &a) == ls::attach_status::ok);
  LS_CHECK(f.reg.announce<ImuSample>("imu", &b) == ls::attach_status::ok);
  LS_CHECK_EQ(a, b);
  LS_CHECK_EQ(f.reg.used(), 1u);

  // ...but only if the layout agrees.
  ls::topic_entry* c = nullptr;
  LS_CHECK(f.reg.announce<CameraFrame>("imu", &c) == ls::attach_status::layout_mismatch);
}

void the_registry_fills_up_and_says_so() {
  fixture f("full");
  for (std::uint32_t i = 0; i < ls::max_topics; ++i) {
    ls::topic_entry* e = nullptr;
    const std::string name = "t" + std::to_string(i);
    LS_CHECK(f.reg.announce<ImuSample>(name, &e) == ls::attach_status::ok);
  }
  ls::topic_entry* overflow = nullptr;
  LS_CHECK(f.reg.announce<ImuSample>("one-too-many", &overflow) ==
           ls::attach_status::registry_full);
}

// Two publishers racing to announce the same topic must produce one entry, not
// two half-written ones.
void concurrent_announce_of_one_name_yields_one_entry() {
  fixture f("race");
  constexpr int kThreads = 8;

  std::atomic<int> ok{0};
  std::vector<std::thread> ts;
  std::vector<ls::topic_entry*> got(kThreads, nullptr);

  for (int t = 0; t < kThreads; ++t) {
    ts.emplace_back([&, t] {
      ls::topic_entry* e = nullptr;
      if (f.reg.announce<ImuSample>("contended", &e) == ls::attach_status::ok) {
        got[t] = e;
        ok.fetch_add(1, std::memory_order_relaxed);
      }
    });
  }
  for (auto& th : ts) th.join();

  LS_CHECK_EQ(ok.load(), kThreads);  // all succeed, as attachers or as creator
  LS_CHECK_EQ(f.reg.used(), 1u);     // but only one slot was consumed

  ls::topic_entry* first = f.reg.find("contended");
  LS_CHECK(first != nullptr);
  for (int t = 0; t < kThreads; ++t) LS_CHECK_EQ(got[t], first);
}

// The ring protocol is only sound with one publisher per topic, so a second
// one has to be refused here rather than left to corrupt the ring.
void a_topic_admits_only_one_publisher() {
  fixture f("exclusive");
  ls::topic_entry* first = nullptr;
  ls::topic_entry* second = nullptr;

  LS_CHECK(f.reg.announce<ImuSample>("imu", &first, true, 1001) ==
           ls::attach_status::ok);

  // A different process claiming the same topic is refused.
  LS_CHECK(f.reg.announce<ImuSample>("imu", &second, true, 1002) ==
           ls::attach_status::already_published);

  // The holder re-announcing is idempotent, so a publisher may re-register.
  LS_CHECK(f.reg.announce<ImuSample>("imu", &first, true, 1001) ==
           ls::attach_status::ok);

  // A subscriber is unaffected: only publishing is exclusive.
  ls::topic_entry* sub = nullptr;
  LS_CHECK(f.reg.attach<ImuSample>("imu", &sub) == ls::attach_status::ok);
  LS_CHECK_EQ(sub, first);
}

}  // namespace

int main() {
  a_topic_admits_only_one_publisher();
  announce_then_find();
  attaching_with_the_right_type_succeeds();
  attaching_with_a_reordered_struct_is_refused();
  attaching_to_a_missing_topic_is_refused();
  announcing_the_same_topic_twice_reuses_the_entry();
  the_registry_fills_up_and_says_so();
  concurrent_announce_of_one_name_yields_one_entry();
  return ls::test::summary("registry");
}
