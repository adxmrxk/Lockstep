// robot/ros2/ros_msgs.hpp : Lockstep equivalents of common ROS 2 messages.
//
// WHY THESE ARE HAND-WRITTEN, WHICH IS THE INTERESTING PART
//
// You cannot generically bridge an arbitrary ROS 2 message into Lockstep, and
// the reason is the whole premise of the project. A ROS message is a generated
// C++ class holding std::vector and std::string members -- both of which own
// heap pointers, both of which LOCKSTEP_MESSAGE refuses by design, because a
// pointer into the publisher's heap means nothing to a subscriber that mapped
// the segment at a different address.
//
// So each bridged type is declared here explicitly, as a fixed-layout struct
// with no indirection, and a conversion function is written for it. That is a
// real cost: a new ROS type is a small code change rather than configuration.
//
// It buys the thing the cost is for. These structs are relocatable, so they
// cross a process boundary with no serialisation and no copy, and they are
// padding-free, so a replay hash over the whole slot reproduces exactly. A
// generic bridge that accepted any ROS type could offer neither.
//
// The types below are the ones a mobile robot actually publishes: an IMU in, a
// velocity command out, a range reading from a proximity sensor.
#pragma once

#include <cstdint>

#include "lockstep/core/message.hpp"

// sensor_msgs/msg/Imu, minus the covariance matrices.
//
// The covariances are 9-element double arrays that most drivers leave filled
// with zeros or -1. Carrying them would triple the message for no information,
// so they are dropped and that is recorded here rather than discovered later by
// someone wondering where they went.
struct RosImu {
  std::uint64_t stamp_ns;      // ROS header stamp, flattened to nanoseconds
  double orientation_x;
  double orientation_y;
  double orientation_z;
  double orientation_w;
  double angular_velocity_x;
  double angular_velocity_y;
  double angular_velocity_z;
  double linear_acceleration_x;
  double linear_acceleration_y;
  double linear_acceleration_z;
  std::uint32_t seq;
  std::uint32_t reserved;
};
LOCKSTEP_MESSAGE(RosImu, stamp_ns, orientation_x, orientation_y, orientation_z,
                 orientation_w, angular_velocity_x, angular_velocity_y,
                 angular_velocity_z, linear_acceleration_x, linear_acceleration_y,
                 linear_acceleration_z, seq, reserved);
static_assert(sizeof(RosImu) == 96, "RosImu should be padding-free");
static_assert(!ls::message_traits<RosImu>::has_padding,
              "padding bytes are indeterminate and would break replay hashing");

// geometry_msgs/msg/Twist. The standard velocity command for a mobile base.
struct RosTwist {
  std::uint64_t stamp_ns;
  double linear_x;
  double linear_y;
  double linear_z;
  double angular_x;
  double angular_y;
  double angular_z;
  std::uint32_t seq;
  std::uint32_t reserved;
};
LOCKSTEP_MESSAGE(RosTwist, stamp_ns, linear_x, linear_y, linear_z, angular_x,
                 angular_y, angular_z, seq, reserved);
static_assert(sizeof(RosTwist) == 64, "RosTwist should be padding-free");
static_assert(!ls::message_traits<RosTwist>::has_padding, "");

// sensor_msgs/msg/Range. What an ultrasonic or IR proximity sensor publishes,
// and the same shape the Elegoo bridge already produces.
struct RosRange {
  std::uint64_t stamp_ns;
  float range_m;
  float min_range_m;
  float max_range_m;
  float field_of_view_rad;
  std::uint32_t radiation_type;  // 0 = ultrasound, 1 = infrared, per REP 145
  std::uint32_t seq;
};
LOCKSTEP_MESSAGE(RosRange, stamp_ns, range_m, min_range_m, max_range_m,
                 field_of_view_rad, radiation_type, seq);
static_assert(sizeof(RosRange) == 32, "RosRange should be padding-free");
static_assert(!ls::message_traits<RosRange>::has_padding, "");

// Default Lockstep topic names for each bridged type. The ROS side is given on
// the command line, because that is what varies between robots.
namespace ros_topics {
inline constexpr const char* imu = "ros/imu";
inline constexpr const char* twist = "ros/cmd_vel";
inline constexpr const char* range = "ros/range";
}  // namespace ros_topics
