// Tests for the ROS 2 bridge conversions.
//
// No ROS graph, no daemon, no discovery, no second process. Every conversion is
// a pure function of its input, so all of this runs in one process in
// milliseconds -- which is the reason the conversions were written as pure
// functions in the first place.
//
// The interesting assertions are the LOSSY ones. It is easy to write a
// round-trip test that passes because it only checks fields that survive. These
// check what is dropped as well as what is kept, so the lossiness is pinned
// rather than discovered by someone wondering where their covariance went.
#include <cmath>
#include <cstdint>
#include <string>

#include "convert.hpp"
#include "ros_msgs.hpp"
#include "support/check.hpp"

namespace {

bool close(double a, double b, double eps = 1e-12) { return std::fabs(a - b) < eps; }

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------
void time_flattens_and_restores() {
  builtin_interfaces::msg::Time t;
  t.sec = 1234;
  t.nanosec = 567890123;

  const std::uint64_t ns = ls::ros::flatten(t);
  LS_CHECK_EQ(ns, 1234567890123ull);

  const auto back = ls::ros::unflatten(ns);
  LS_CHECK_EQ(back.sec, 1234);
  LS_CHECK_EQ(back.nanosec, 567890123u);
}

// A negative stamp is a driver bug. Flattening it into an unsigned nanosecond
// count would produce an enormous positive number, which would then look like a
// timestamp from the year 2500 rather than like the bug it is.
void negative_time_does_not_wrap() {
  builtin_interfaces::msg::Time t;
  t.sec = -5;
  t.nanosec = 0;
  LS_CHECK_EQ(ls::ros::flatten(t), 0ull);
}

void time_round_trips_over_a_range() {
  for (std::int32_t s : {0, 1, 59, 3600, 100000, 2000000000}) {
    for (std::uint32_t n : {0u, 1u, 999999999u}) {
      builtin_interfaces::msg::Time t;
      t.sec = s;
      t.nanosec = n;
      const auto back = ls::ros::unflatten(ls::ros::flatten(t));
      LS_CHECK_EQ(back.sec, s);
      LS_CHECK_EQ(back.nanosec, n);
    }
  }
}

// ---------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------
sensor_msgs::msg::Imu make_imu() {
  sensor_msgs::msg::Imu m;
  m.header.stamp.sec = 42;
  m.header.stamp.nanosec = 500000000;
  m.header.frame_id = "imu_link";
  m.orientation.x = 0.1; m.orientation.y = 0.2;
  m.orientation.z = 0.3; m.orientation.w = 0.4;
  m.angular_velocity.x = 1.5; m.angular_velocity.y = -2.5; m.angular_velocity.z = 3.5;
  m.linear_acceleration.x = 9.81; m.linear_acceleration.y = 0.01;
  m.linear_acceleration.z = -0.02;
  for (int i = 0; i < 9; ++i) m.orientation_covariance[i] = 0.5;
  return m;
}

void imu_carries_every_field_it_claims_to() {
  const auto src = make_imu();
  const RosImu ls_msg = ls::ros::from_ros(src, 7);

  LS_CHECK_EQ(ls_msg.stamp_ns, 42500000000ull);
  LS_CHECK_EQ(ls_msg.seq, 7u);
  LS_CHECK(close(ls_msg.orientation_x, 0.1));
  LS_CHECK(close(ls_msg.orientation_w, 0.4));
  LS_CHECK(close(ls_msg.angular_velocity_y, -2.5));
  LS_CHECK(close(ls_msg.linear_acceleration_x, 9.81));
  LS_CHECK(close(ls_msg.linear_acceleration_z, -0.02));

  const auto back = ls::ros::to_ros(ls_msg, "imu_link");
  LS_CHECK_EQ(back.header.stamp.sec, 42);
  LS_CHECK_EQ(back.header.stamp.nanosec, 500000000u);
  LS_CHECK(back.header.frame_id == std::string("imu_link"));
  LS_CHECK(close(back.orientation.x, 0.1));
  LS_CHECK(close(back.angular_velocity.z, 3.5));
  LS_CHECK(close(back.linear_acceleration.x, 9.81));
}

// The documented loss. Covariance does not survive, and the rebuilt message
// must say "unknown" (-1) rather than leave 0, which per REP 145 would assert
// the covariance is known to be exactly zero -- a different and false claim.
void imu_covariance_is_dropped_and_marked_unknown() {
  const auto src = make_imu();
  LS_CHECK(close(src.orientation_covariance[0], 0.5));  // it was set

  const RosImu ls_msg = ls::ros::from_ros(src, 0);
  const auto back = ls::ros::to_ros(ls_msg, "imu_link");

  LS_CHECK(close(back.orientation_covariance[0], -1.0));
  LS_CHECK(close(back.angular_velocity_covariance[0], -1.0));
  LS_CHECK(close(back.linear_acceleration_covariance[0], -1.0));
}

// ---------------------------------------------------------------------------
// Twist -- the lossless one
// ---------------------------------------------------------------------------
void twist_round_trips_exactly() {
  geometry_msgs::msg::Twist src;
  src.linear.x = 0.25; src.linear.y = -0.5; src.linear.z = 0.125;
  src.angular.x = 1.0; src.angular.y = -1.5; src.angular.z = 2.25;

  const RosTwist ls_msg = ls::ros::from_ros(src, 99999ull, 3);
  LS_CHECK_EQ(ls_msg.stamp_ns, 99999ull);
  LS_CHECK_EQ(ls_msg.seq, 3u);

  const auto back = ls::ros::to_ros(ls_msg);
  LS_CHECK(close(back.linear.x, 0.25));
  LS_CHECK(close(back.linear.y, -0.5));
  LS_CHECK(close(back.linear.z, 0.125));
  LS_CHECK(close(back.angular.x, 1.0));
  LS_CHECK(close(back.angular.y, -1.5));
  LS_CHECK(close(back.angular.z, 2.25));
}

// ---------------------------------------------------------------------------
// Range
// ---------------------------------------------------------------------------
void range_round_trips_and_loses_only_the_frame() {
  sensor_msgs::msg::Range src;
  src.header.stamp.sec = 10;
  src.header.stamp.nanosec = 250000000;
  src.header.frame_id = "sonar_front";
  src.range = 1.25f;
  src.min_range = 0.02f;
  src.max_range = 4.0f;
  src.field_of_view = 0.26f;
  src.radiation_type = 0;  // ultrasound

  const RosRange ls_msg = ls::ros::from_ros(src, 5);
  LS_CHECK_EQ(ls_msg.stamp_ns, 10250000000ull);
  LS_CHECK_EQ(ls_msg.seq, 5u);
  LS_CHECK(close(ls_msg.range_m, 1.25, 1e-6));
  LS_CHECK(close(ls_msg.max_range_m, 4.0, 1e-6));
  LS_CHECK_EQ(ls_msg.radiation_type, 0u);

  // frame_id is a variable-length string and cannot live in a relocatable
  // message, so it is supplied on the way back rather than carried.
  const auto back = ls::ros::to_ros(ls_msg, "sonar_front");
  LS_CHECK(back.header.frame_id == std::string("sonar_front"));
  LS_CHECK(close(back.range, 1.25, 1e-6));
  LS_CHECK_EQ(back.header.stamp.sec, 10);
}

// ---------------------------------------------------------------------------
// The property that makes any of this possible
// ---------------------------------------------------------------------------
void bridged_types_can_actually_cross_a_segment() {
  // If any of these failed, the type could not be published at all -- a ROS
  // message cannot go on the bus until it has been flattened into something
  // relocatable, which is the entire reason convert.hpp exists.
  LS_CHECK(ls::message_traits<RosImu>::declared);
  LS_CHECK(ls::message_traits<RosTwist>::declared);
  LS_CHECK(ls::message_traits<RosRange>::declared);

  LS_CHECK(ls::is_relocatable_v<RosImu>);
  LS_CHECK(ls::is_relocatable_v<RosTwist>);
  LS_CHECK(ls::is_relocatable_v<RosRange>);

  // Padding is indeterminate; a replay hash covers the whole slot.
  LS_CHECK(!ls::message_traits<RosImu>::has_padding);
  LS_CHECK(!ls::message_traits<RosTwist>::has_padding);
  LS_CHECK(!ls::message_traits<RosRange>::has_padding);

  // Distinct types must not be confusable at connect time.
  LS_CHECK(ls::message_traits<RosImu>::layout_hash !=
           ls::message_traits<RosTwist>::layout_hash);
  LS_CHECK(ls::message_traits<RosTwist>::layout_hash !=
           ls::message_traits<RosRange>::layout_hash);
}

}  // namespace

int main() {
  time_flattens_and_restores();
  negative_time_does_not_wrap();
  time_round_trips_over_a_range();
  imu_carries_every_field_it_claims_to();
  imu_covariance_is_dropped_and_marked_unknown();
  twist_round_trips_exactly();
  range_round_trips_and_loses_only_the_frame();
  bridged_types_can_actually_cross_a_segment();
  return ls::test::summary("ros_bridge");
}
