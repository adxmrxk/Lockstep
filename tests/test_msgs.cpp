// Tests for the standard robotics message set.
//
// Most of what matters here is checked at compile time by the static_asserts in
// the header, so the fact that this file BUILDS is already most of the test.
// What is left is the part a static_assert cannot reach: that the types are
// usable on a real bus, that they are distinguishable from each other at
// connect time, and that a message survives a round trip through shared memory
// with its values intact.
#include <cstdint>
#include <string>

#include "lockstep/msgs/robotics.hpp"
#include "lockstep/pubsub.hpp"
#include "support/check.hpp"

namespace {

using namespace ls::msgs;

// The padding rule, restated as a runtime check so a failure names the type
// rather than pointing at a line in a header.
void every_type_is_replay_safe() {
  LS_CHECK(!ls::message_traits<Vec3>::has_padding);
  LS_CHECK(!ls::message_traits<Quat>::has_padding);
  LS_CHECK(!ls::message_traits<Imu>::has_padding);
  LS_CHECK(!ls::message_traits<Range>::has_padding);
  LS_CHECK(!ls::message_traits<BatteryState>::has_padding);
  LS_CHECK(!ls::message_traits<Pose>::has_padding);
  LS_CHECK(!ls::message_traits<Twist>::has_padding);
  LS_CHECK(!ls::message_traits<Odometry>::has_padding);
  LS_CHECK(!ls::message_traits<JointState>::has_padding);
  LS_CHECK(!ls::message_traits<Transform>::has_padding);

  LS_CHECK(ls::message_traits<Imu>::declared);
  LS_CHECK(ls::message_traits<Odometry>::declared);
  LS_CHECK(ls::message_traits<Transform>::declared);
}

// A layout hash collision between two of these would let a node publishing
// Twist connect to a subscriber expecting Odometry, and the subscriber would
// read 64 bytes of one as 120 bytes of the other.
void no_two_types_share_a_layout_hash() {
  const std::uint64_t hashes[] = {
      ls::message_traits<Vec3>::layout_hash,
      ls::message_traits<Quat>::layout_hash,
      ls::message_traits<Imu>::layout_hash,
      ls::message_traits<Range>::layout_hash,
      ls::message_traits<BatteryState>::layout_hash,
      ls::message_traits<Pose>::layout_hash,
      ls::message_traits<Twist>::layout_hash,
      ls::message_traits<Odometry>::layout_hash,
      ls::message_traits<JointState>::layout_hash,
      ls::message_traits<Transform>::layout_hash,
  };
  constexpr std::size_t n = sizeof(hashes) / sizeof(hashes[0]);
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t k = i + 1; k < n; ++k) {
      LS_CHECK(hashes[i] != hashes[k]);
    }
  }
}

// Pose and Twist are both "stamp, two 8-byte-aligned blobs, seq, reserved".
// They must still be told apart, which is what makes the hash worth having:
// it covers field NAMES and types, not just the shape of the memory.
void types_of_the_same_shape_are_still_distinct() {
  LS_CHECK(ls::message_traits<Vec3>::layout_hash !=
           ls::message_traits<Quat>::layout_hash);
  LS_CHECK(ls::message_traits<Twist>::layout_hash !=
           ls::message_traits<Pose>::layout_hash);
}

// The real test: through shared memory, out the other side, values intact.
void a_message_survives_the_bus() {
  const std::string name = "lockstep-test-msgs";
  ls::segment::unlink(name);
  ls::bus b = ls::bus::create(name, {{64, 64}, {512, 32}, {4096, 16}});

  auto odom_pub = ls::publisher<Odometry>::create(b, topics::odom, 8);
  auto odom_sub = ls::subscriber<Odometry>::attach(b, topics::odom);

  {
    auto ln = odom_pub.loan_message();
    ln->stamp_ns = 1234567890123ull;
    ln->position = Vec3{1.5, -2.25, 0.125};
    ln->orientation = Quat{0.0, 0.0, 0.7071067811865476, 0.7071067811865476};
    ln->linear_velocity = Vec3{0.4, 0.0, 0.0};
    ln->angular_velocity = Vec3{0.0, 0.0, -0.2};
    ln->seq = 7;
    ln->reserved = 0;
    odom_pub.publish(ln);
  }

  ls::sample<Odometry> s;
  LS_CHECK(odom_sub.take(s) == ls::read_result::ok);
  Odometry got{};
  LS_CHECK(s.copy_out(got));
  LS_CHECK_EQ(got.stamp_ns, 1234567890123ull);
  LS_CHECK_EQ(got.seq, 7u);
  LS_CHECK(got.position.x == 1.5);
  LS_CHECK(got.position.y == -2.25);
  LS_CHECK(got.orientation.w == 0.7071067811865476);
  LS_CHECK(got.linear_velocity.x == 0.4);
  LS_CHECK(got.angular_velocity.z == -0.2);

  ls::segment::unlink(name);
}

// Frame ids are the one variable-length thing that survives, so check they
// actually round trip and that an over-long one is truncated rather than
// writing past the field.
void transform_frame_ids_round_trip() {
  const std::string name = "lockstep-test-msgs-tf";
  ls::segment::unlink(name);
  ls::bus b = ls::bus::create(name, {{64, 64}, {512, 32}, {4096, 16}});

  auto pub = ls::publisher<Transform>::create(b, topics::tf, 8);
  auto sub = ls::subscriber<Transform>::attach(b, topics::tf);

  {
    auto ln = pub.loan_message();
    *ln.get() = Transform{};
    LS_CHECK(ln->parent.assign("odom"));
    LS_CHECK(ln->child.assign("base_link"));
    ln->translation = Vec3{1.0, 2.0, 3.0};
    pub.publish(ln);
  }

  ls::sample<Transform> s;
  LS_CHECK(sub.take(s) == ls::read_result::ok);
  Transform got{};
  LS_CHECK(s.copy_out(got));
  LS_CHECK(got.parent.view() == "odom");
  LS_CHECK(got.child.view() == "base_link");
  LS_CHECK(got.translation.z == 3.0);

  // Longer than the field. inline_string REFUSES rather than truncating, which
  // is the right call: a silently shortened frame id would name a different
  // frame, and a transform tree would quietly reparent itself.
  frame_id f;
  LS_CHECK(f.assign("base_link"));
  LS_CHECK(!f.assign(std::string(200, 'x')));
  LS_CHECK(f.view() == "base_link");   // the failed assign changed nothing

  ls::segment::unlink(name);
}

// kMaxJoints is a hard edge, so check the last slot is reachable and that
// `count` is what callers are meant to trust.
void joint_state_uses_its_whole_array() {
  JointState js{};
  js.count = static_cast<std::uint32_t>(kMaxJoints);
  for (std::size_t i = 0; i < kMaxJoints; ++i) {
    js.position[i] = static_cast<double>(i);
    js.velocity[i] = static_cast<double>(i) * 0.5;
    js.effort[i] = -static_cast<double>(i);
  }
  LS_CHECK(js.position[kMaxJoints - 1] == static_cast<double>(kMaxJoints - 1));
  LS_CHECK(js.velocity[kMaxJoints - 1] == static_cast<double>(kMaxJoints - 1) * 0.5);
  LS_CHECK(js.effort[0] == 0.0);
  LS_CHECK_EQ(js.count, static_cast<std::uint32_t>(kMaxJoints));
}

// A bus whose pools are too small for a ring has to say so, not corrupt
// something.
//
// Worth a test because of how this presented while the above was being
// written: the pool list was one class short, the ring allocation failed, and
// MSVC reported the uncaught exception as exit code 0xC0000409, which is named
// STATUS_STACK_BUFFER_OVERRUN. There was no stack overrun. __fastfail reuses
// that status for several fatal conditions, so the number sends you hunting a
// memory bug that is not there. Catching the throw here states what actually
// happens.
void a_bus_too_small_for_a_ring_says_so() {
  const std::string name = "lockstep-test-msgs-small";
  ls::segment::unlink(name);
  // Nothing here can hold a ring of 8, and Transform needs 136 bytes a slot.
  ls::bus b = ls::bus::create(name, {{64, 8}});

  bool threw = false;
  try {
    auto pub = ls::publisher<Transform>::create(b, topics::tf, 8);
    (void)pub;
  } catch (const std::exception&) {
    threw = true;
  }
  LS_CHECK(threw);
  ls::segment::unlink(name);
}

}  // namespace

int main() {
  every_type_is_replay_safe();
  no_two_types_share_a_layout_hash();
  types_of_the_same_shape_are_still_distinct();
  joint_state_uses_its_whole_array();
  a_message_survives_the_bus();
  transform_frame_ids_round_trip();
  a_bus_too_small_for_a_ring_says_so();
  return ls::test::summary("msgs");
}
