// loan / publish / subscribe.
//
// The claim under test is "zero copy", so the central assertion is an identity
// one: the address the subscriber reads through must be the same segment
// location the publisher wrote through. If a copy had crept onto the path, the
// offsets would differ.
#include <cstdint>
#include <string>
#include <unistd.h>

#include "lockstep/pubsub.hpp"
#include "support/check.hpp"
#include "support/demo_msgs.hpp"

namespace {

std::string unique_name(const char* tag) {
  return std::string("lockstep-ps-") + tag + "-" + std::to_string(::getpid());
}

ls::bus make_bus(const char* tag) {
  const std::string name = unique_name(tag);
  ls::segment::unlink(name);
  return ls::bus::create(name, {{64, 64}, {256, 64}, {4096, 32}, {64 * 1024, 8}});
}

void a_published_message_comes_back() {
  ls::bus b = make_bus("roundtrip");
  auto pub = ls::publisher<ImuSample>::create(b, "imu/raw", 8);
  auto sub = ls::subscriber<ImuSample>::attach(b, "imu/raw");

  auto ln = pub.loan_message();
  LS_CHECK(ln.valid());
  ln->stamp_ns = 42;
  ln->ax = 1.5f;
  ln->gz = -2.5f;
  LS_CHECK(pub.publish(ln));

  ls::sample<ImuSample> s;
  LS_CHECK(sub.take(s) == ls::read_result::ok);
  LS_CHECK_EQ(s->stamp_ns, 42u);
  LS_CHECK_EQ(s->ax, 1.5f);
  LS_CHECK_EQ(s->gz, -2.5f);
  LS_CHECK_EQ(s.sequence(), 0u);
  LS_CHECK(s.still_valid());

  LS_CHECK(sub.take(s) == ls::read_result::empty);
}

// The zero-copy claim, stated as an address identity.
void the_subscriber_reads_the_publishers_bytes() {
  ls::bus b = make_bus("zerocopy");
  auto pub = ls::publisher<ImuSample>::create(b, "imu", 8);
  auto sub = ls::subscriber<ImuSample>::attach(b, "imu");

  auto ln = pub.loan_message();
  const void* written_to = ln.get();
  ln->stamp_ns = 7;
  pub.publish(ln);

  ls::sample<ImuSample> s;
  LS_CHECK(sub.take(s) == ls::read_result::ok);
  const void* read_from = s.get();

  // Same process here, so the pointers are literally equal. Across processes
  // the bases differ and the segment OFFSETS are what match -- that is what
  // test_pubsub_child checks.
  LS_CHECK_EQ(written_to, read_from);
  LS_CHECK(b.seg().contains(read_from, sizeof(ImuSample)));

  // And mutating through the publisher's pointer is visible through the
  // subscriber's, which no copy-based transport could do.
  const_cast<ImuSample*>(s.get())->stamp_ns = 99;
  LS_CHECK_EQ(static_cast<const ImuSample*>(written_to)->stamp_ns, 99u);
}

void every_subscriber_gets_every_message() {
  ls::bus b = make_bus("broadcast");
  auto pub = ls::publisher<ImuSample>::create(b, "imu", 8);
  auto s1 = ls::subscriber<ImuSample>::attach(b, "imu");
  auto s2 = ls::subscriber<ImuSample>::attach(b, "imu");

  for (std::uint64_t i = 0; i < 5; ++i) {
    auto ln = pub.loan_message();
    ln->stamp_ns = i;
    pub.publish(ln);
  }

  for (auto* sub : {&s1, &s2}) {
    for (std::uint64_t i = 0; i < 5; ++i) {
      ls::sample<ImuSample> s;
      LS_CHECK(sub->take(s) == ls::read_result::ok);
      LS_CHECK_EQ(s->stamp_ns, i);
    }
    LS_CHECK_EQ(sub->received(), 5u);
  }
}

// A large payload rides in the arena with a shm_span pointing at it, and the
// message itself stays 64 bytes.
void a_big_payload_travels_out_of_line() {
  ls::bus b = make_bus("body");
  constexpr std::size_t kBody = 4096;

  auto pub = ls::publisher<CameraFrame>::create(b, "cam", 4, kBody);
  auto sub = ls::subscriber<CameraFrame>::attach(b, "cam");

  auto ln = pub.loan_message();
  LS_CHECK(ln.body() != nullptr);
  LS_CHECK_EQ(ln.body_capacity(), kBody);

  ln->stamp_ns = 5;
  ln->width = 64;
  ln->height = 64;
  LS_CHECK(ln->frame_id.assign("front"));
  for (std::size_t i = 0; i < kBody; ++i)
    ln.body()[i] = static_cast<std::uint8_t>(i & 0xff);
  ln->pixels.bind(ln.body(), kBody);
  ln.set_body_size(kBody);
  pub.publish(ln);

  ls::sample<CameraFrame> s;
  LS_CHECK(sub.take(s) == ls::read_result::ok);
  LS_CHECK_EQ(s->width, 64u);
  LS_CHECK(s->frame_id == std::string_view("front"));
  LS_CHECK_EQ(s->pixels.size(), kBody);
  LS_CHECK_EQ(s.body_size(), kBody);

  // The span must resolve to the same bytes, and the message must still be the
  // 64-byte descriptor the README promises rather than growing with the image.
  LS_CHECK_EQ(sizeof(CameraFrame), 64u);
  LS_CHECK_EQ(s->pixels[0], 0u);
  LS_CHECK_EQ(s->pixels[255], 255u);
  LS_CHECK_EQ(static_cast<const void*>(s->pixels.data()),
              static_cast<const void*>(s.body()));
}

void a_slow_subscriber_is_told_it_was_lapped() {
  ls::bus b = make_bus("overrun");
  auto pub = ls::publisher<ImuSample>::create(b, "imu", 4);
  auto sub = ls::subscriber<ImuSample>::attach(b, "imu");

  for (std::uint64_t i = 0; i < 12; ++i) {
    auto ln = pub.loan_message();
    ln->stamp_ns = i;
    pub.publish(ln);
  }

  ls::sample<ImuSample> s;
  const auto rr = sub.take(s);
  LS_CHECK(rr == ls::read_result::overrun);
  LS_CHECK_EQ(sub.overruns(), 1u);

  LS_CHECK(sub.take(s) == ls::read_result::ok);
  LS_CHECK_EQ(s->stamp_ns, 8u);  // oldest still resident: 12 - 4
}

// A sample held across enough publications to lap the ring must report itself
// stale rather than quietly returning the newer message's bytes.
void a_stale_sample_knows_it_is_stale() {
  ls::bus b = make_bus("stale");
  auto pub = ls::publisher<ImuSample>::create(b, "imu", 2);
  auto sub = ls::subscriber<ImuSample>::attach(b, "imu");

  auto first = pub.loan_message();
  first->stamp_ns = 1;
  pub.publish(first);

  ls::sample<ImuSample> s;
  LS_CHECK(sub.take(s) == ls::read_result::ok);
  LS_CHECK(s.still_valid());

  ImuSample copy{};
  LS_CHECK(s.copy_out(copy));
  LS_CHECK_EQ(copy.stamp_ns, 1u);

  // Lap the 2-slot ring so the held sample's slot is reused.
  for (int i = 0; i < 4; ++i) {
    auto ln = pub.loan_message();
    ln->stamp_ns = 100 + i;
    pub.publish(ln);
  }

  LS_CHECK(!s.still_valid());
  ImuSample after{};
  LS_CHECK(!s.copy_out(after));  // the copy is refused, not silently wrong
}

void a_second_publisher_on_a_topic_is_refused() {
  ls::bus b = make_bus("onepub");
  auto pub = ls::publisher<ImuSample>::create(b, "imu", 4);

  // Same process, same pid, so re-creating is the idempotent case and works.
  auto again = ls::publisher<ImuSample>::create(b, "imu", 4);
  LS_CHECK_EQ(again.topic(), pub.topic());

  // A different pid is what gets refused; that path is covered in
  // test_registry (a_topic_admits_only_one_publisher) where the pid can be
  // supplied directly.
  LS_CHECK(pub.topic()->publisher_pid.load() ==
           ls::publisher<ImuSample>::current_pid());
}

void subscribing_to_a_topic_with_no_publisher_fails() {
  ls::bus b = make_bus("nopub");
  bool threw = false;
  try {
    (void)ls::subscriber<ImuSample>::attach(b, "never/published");
  } catch (const std::exception&) {
    threw = true;
  }
  LS_CHECK(threw);
}

}  // namespace

int main() {
  a_published_message_comes_back();
  the_subscriber_reads_the_publishers_bytes();
  every_subscriber_gets_every_message();
  a_big_payload_travels_out_of_line();
  a_slow_subscriber_is_told_it_was_lapped();
  a_stale_sample_knows_it_is_stale();
  a_second_publisher_on_a_topic_is_refused();
  subscribing_to_a_topic_with_no_publisher_fails();
  return ls::test::summary("pubsub");
}
