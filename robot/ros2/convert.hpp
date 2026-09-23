// robot/ros2/convert.hpp : ROS 2 message <-> Lockstep message.
//
// Pure functions, deliberately. Every one takes a message and returns a
// message, touches no bus, no node, and no clock, which is what lets
// test_ros_bridge check them exhaustively without a ROS graph running.
//
// The conversions are lossy in one direction and that is documented per type
// rather than discovered. Nothing here silently drops a field.
#pragma once

#include <cstdint>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "ros_msgs.hpp"

namespace ls::ros {

// ROS splits time into seconds and nanoseconds. Lockstep messages carry one
// 64-bit nanosecond count, because a split stamp is two fields that can
// disagree and a replay has to reproduce exactly one number.
inline std::uint64_t flatten(const builtin_interfaces::msg::Time& t) {
  const std::int64_t s = static_cast<std::int64_t>(t.sec);
  if (s < 0) return 0;  // pre-epoch stamps are a driver bug, not a time
  return static_cast<std::uint64_t>(s) * 1000000000ull +
         static_cast<std::uint64_t>(t.nanosec);
}

inline builtin_interfaces::msg::Time unflatten(std::uint64_t ns) {
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<std::int32_t>(ns / 1000000000ull);
  t.nanosec = static_cast<std::uint32_t>(ns % 1000000000ull);
  return t;
}

// ---------------------------------------------------------------------- IMU
// LOSSY: the three 9-element covariance matrices are dropped. Most drivers
// publish them as all-zero or leading -1 (meaning "unknown"), so carrying them
// would triple the message to move no information.
inline RosImu from_ros(const sensor_msgs::msg::Imu& m, std::uint32_t seq) {
  RosImu o{};
  o.stamp_ns = flatten(m.header.stamp);
  o.seq = seq;
  o.reserved = 0;
  o.orientation_x = m.orientation.x;
  o.orientation_y = m.orientation.y;
  o.orientation_z = m.orientation.z;
  o.orientation_w = m.orientation.w;
  o.angular_velocity_x = m.angular_velocity.x;
  o.angular_velocity_y = m.angular_velocity.y;
  o.angular_velocity_z = m.angular_velocity.z;
  o.linear_acceleration_x = m.linear_acceleration.x;
  o.linear_acceleration_y = m.linear_acceleration.y;
  o.linear_acceleration_z = m.linear_acceleration.z;
  return o;
}

inline sensor_msgs::msg::Imu to_ros(const RosImu& m, const std::string& frame_id) {
  sensor_msgs::msg::Imu o;
  o.header.stamp = unflatten(m.stamp_ns);
  o.header.frame_id = frame_id;
  o.orientation.x = m.orientation_x;
  o.orientation.y = m.orientation_y;
  o.orientation.z = m.orientation_z;
  o.orientation.w = m.orientation_w;
  o.angular_velocity.x = m.angular_velocity_x;
  o.angular_velocity.y = m.angular_velocity_y;
  o.angular_velocity.z = m.angular_velocity_z;
  o.linear_acceleration.x = m.linear_acceleration_x;
  o.linear_acceleration.y = m.linear_acceleration_y;
  o.linear_acceleration.z = m.linear_acceleration_z;
  // Covariance was not carried across, so say "unknown" rather than claim zero,
  // which per REP 145 would mean "known to be exactly zero".
  o.orientation_covariance[0] = -1.0;
  o.angular_velocity_covariance[0] = -1.0;
  o.linear_acceleration_covariance[0] = -1.0;
  return o;
}

// -------------------------------------------------------------------- Twist
// LOSSLESS. Twist has no header, so the stamp is supplied by the caller.
inline RosTwist from_ros(const geometry_msgs::msg::Twist& m, std::uint64_t stamp_ns,
                         std::uint32_t seq) {
  RosTwist o{};
  o.stamp_ns = stamp_ns;
  o.seq = seq;
  o.reserved = 0;
  o.linear_x = m.linear.x;
  o.linear_y = m.linear.y;
  o.linear_z = m.linear.z;
  o.angular_x = m.angular.x;
  o.angular_y = m.angular.y;
  o.angular_z = m.angular.z;
  return o;
}

inline geometry_msgs::msg::Twist to_ros(const RosTwist& m) {
  geometry_msgs::msg::Twist o;
  o.linear.x = m.linear_x;
  o.linear.y = m.linear_y;
  o.linear.z = m.linear_z;
  o.angular.x = m.angular_x;
  o.angular.y = m.angular_y;
  o.angular.z = m.angular_z;
  return o;
}

// -------------------------------------------------------------------- Range
// LOSSY: header.frame_id is dropped, because a variable-length string cannot
// live in a relocatable message. It is restored from a fixed frame on the way
// back out, which is correct for a single-sensor bridge and wrong for one
// multiplexing several -- so the bridge takes one frame per topic.
inline RosRange from_ros(const sensor_msgs::msg::Range& m, std::uint32_t seq) {
  RosRange o{};
  o.stamp_ns = flatten(m.header.stamp);
  o.seq = seq;
  o.range_m = m.range;
  o.min_range_m = m.min_range;
  o.max_range_m = m.max_range;
  o.field_of_view_rad = m.field_of_view;
  o.radiation_type = m.radiation_type;
  return o;
}

inline sensor_msgs::msg::Range to_ros(const RosRange& m, const std::string& frame_id) {
  sensor_msgs::msg::Range o;
  o.header.stamp = unflatten(m.stamp_ns);
  o.header.frame_id = frame_id;
  o.range = m.range_m;
  o.min_range = m.min_range_m;
  o.max_range = m.max_range_m;
  o.field_of_view = m.field_of_view_rad;
  o.radiation_type = static_cast<std::uint8_t>(m.radiation_type);
  return o;
}

}  // namespace ls::ros
