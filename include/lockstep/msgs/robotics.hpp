// msgs/robotics.hpp : the messages nearly every robot has.
//
// Without these, putting Lockstep on a new robot means designing your own
// message types first, getting the padding right, and finding out at runtime if
// you did not. That is a real barrier for something whose whole pitch is "run
// one more process". This is the set you pick from instead.
//
// They are modelled on the ROS 2 common_interfaces, deliberately, so the
// mapping is obvious and the ROS bridge has somewhere to land. They are NOT
// copies: a ROS message holds std::vector and std::string, which are heap
// pointers and meaningless in another process, so every variable-length field
// here is either dropped, fixed-capacity, or an arena span.
//
// THE PADDING RULE, which is the thing to actually be careful about:
//
// A replay hash is taken over the whole message slot. Padding bytes are
// indeterminate, so a type with interior padding reproduces only if the
// publisher zero-fills it first. Rather than rely on remembering that, every
// type here is laid out so sizeof() lands on a multiple of 8 with no interior
// padding at all, and a static_assert holds it that way. If you add a field and
// the build breaks, that is the assert doing its job: reorder, or add an
// explicit reserved field, do not relax the check.
//
// WHAT IS NOT HERE
//
// Anything genuinely unbounded. A point cloud or a camera image belongs in the
// arena with a span pointing at it, the way the CameraFrame example in the
// README does it, because a 24 MB payload has no business inside a fixed-size
// ring slot. The pattern is ls::shm_span<T>; there is no point wrapping it in a
// type here that would only constrain it.
#pragma once

#include <cstdint>

#include "lockstep/containers/inline_string.hpp"
#include "lockstep/core/message.hpp"

// A NOTE ON THE SHAPE OF THIS FILE
//
// The types live in ls::msgs, but every LOCKSTEP_MESSAGE sits at global scope
// further down instead of next to its struct. That is not a style choice: the
// macro expands to `namespace ls { template <> struct padding_bytes<T> ... }`,
// and opening `namespace ls` from inside `ls::msgs` would declare
// `ls::msgs::ls`, which is not the same namespace and does not compile.
//
// The namespace is worth the awkwardness. `Range` and `Imu` at global scope
// would collide with the ROS headers that the bridge in robot/ros2 includes.

namespace ls::msgs {

// Frame ids are short by convention ("base_link", "odom", "laser"). 28 bytes
// plus the 4 byte length lands the type on 32 with nothing wasted.
inline constexpr std::size_t kFrameIdCapacity = 28;
using frame_id = ls::inline_string<kFrameIdCapacity>;

// Joints per message. Fixed because a ring slot is fixed; 16 covers a 6-axis
// arm, a quadruped, or a differential base with room over.
inline constexpr std::size_t kMaxJoints = 16;

// ---------------------------------------------------------------------------
// Geometry primitives
//
// Declared messages in their own right, so they can nest inside the types below
// and still satisfy the relocatable contract. Plain aggregates do not: class
// types are not relocatable unless declared, which is the default that keeps a
// std::string from ever reaching shared memory.
//
// double rather than float throughout, matching ROS. For position it is not
// optional: float has about 7 significant digits, so a robot 10 km from its
// origin would be quantised to centimetres.
// ---------------------------------------------------------------------------

struct Vec3 {
  double x;
  double y;
  double z;
};

// Orientation as a quaternion, not Euler angles. Euler needs a stated
// convention, and two nodes that disagree about it fail in a way that looks
// like a sensor fault.
struct Quat {
  double x;
  double y;
  double z;
  double w;
};

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------

// Inertial measurement. `valid` is a bitmask because an IMU commonly delivers
// rates and accelerations while its orientation estimate is still converging,
// and a consumer needs to tell "zero rotation" from "no idea yet".
struct Imu {
  std::uint64_t stamp_ns;
  Vec3 angular_velocity;        // rad/s
  Vec3 linear_acceleration;     // m/s^2
  Quat orientation;
  std::uint32_t seq;
  std::uint32_t valid;          // see imu_valid
};

namespace imu_valid {
inline constexpr std::uint32_t angular_velocity = 1u << 0;
inline constexpr std::uint32_t linear_acceleration = 1u << 1;
inline constexpr std::uint32_t orientation = 1u << 2;
}  // namespace imu_valid

// A single distance reading, from an ultrasonic, a ToF sensor or one lidar ray.
//
// `valid` is separate from the distance rather than encoded as a sentinel,
// because every sensor of this kind picks a different sentinel (0, max range,
// -1) and control code that steers on the wrong one jerks for no reason. The
// driver decides validity once, here.
struct Range {
  std::uint64_t stamp_ns;
  float distance_m;
  float min_m;
  float max_m;
  std::uint32_t seq;
  std::uint32_t valid;          // 0 = ignore distance_m
  std::uint32_t reserved;
};

// Battery, because "the robot got slow and then stopped" is a power problem
// about as often as it is a software one, and a journal that does not record
// the voltage cannot tell you which.
struct BatteryState {
  std::uint64_t stamp_ns;
  float voltage_v;
  float current_a;              // negative while discharging, as in ROS
  float charge_ah;
  float capacity_ah;
  float percentage;             // 0 to 1
  float temperature_c;
  std::uint32_t seq;
  std::uint32_t present;        // 0 = no battery detected
};

// ---------------------------------------------------------------------------
// State and commands
// ---------------------------------------------------------------------------

// Where the robot thinks it is.
struct Pose {
  std::uint64_t stamp_ns;
  Vec3 position;                // metres
  Quat orientation;
  std::uint32_t seq;
  std::uint32_t reserved;
};

// How fast to move, or how fast it is moving. The usual velocity command.
struct Twist {
  std::uint64_t stamp_ns;
  Vec3 linear;                  // m/s
  Vec3 angular;                 // rad/s
  std::uint32_t seq;
  std::uint32_t reserved;
};

// Pose and velocity together, which is what a wheel or visual odometer
// produces and what a localiser consumes.
//
// Covariance is deliberately absent. It is 72 doubles for the pair, it would
// quadruple the message, and almost nothing consumes it. A node that needs it
// should define its own type rather than make every robot carry 576 bytes of
// zeros.
struct Odometry {
  std::uint64_t stamp_ns;
  Vec3 position;
  Quat orientation;
  Vec3 linear_velocity;
  Vec3 angular_velocity;
  std::uint32_t seq;
  std::uint32_t reserved;
};

// Joint positions, velocities and efforts, up to kMaxJoints.
//
// No names. ROS carries a string per joint, which cannot cross a shared-memory
// boundary, and the fixed-capacity alternative would add 512 bytes to every
// message to re-send constants that never change. Joint ORDER is the contract
// instead, declared once wherever the robot is configured, which is what
// control code ends up relying on anyway.
struct JointState {
  std::uint64_t stamp_ns;
  double position[kMaxJoints];  // rad, or m for a prismatic joint
  double velocity[kMaxJoints];
  double effort[kMaxJoints];    // Nm, or N
  std::uint32_t count;          // how many of the arrays are meaningful
  std::uint32_t seq;
};

// One edge of a transform tree, the equivalent of a single ROS TF.
//
// Frame ids are fixed-capacity strings rather than dropped, because a transform
// without its two frame names is not a transform, it is six numbers.
struct Transform {
  std::uint64_t stamp_ns;
  Vec3 translation;
  Quat rotation;
  frame_id parent;
  frame_id child;
  std::uint32_t seq;
  std::uint32_t reserved;
};

// ---------------------------------------------------------------------------
// Topic names
//
// Conventions, not requirements, in one place so two nodes agree by including a
// header rather than by both spelling the same string correctly. That bug costs
// an afternoon every time, because a subscriber on a topic nobody publishes
// looks exactly like a publisher that is broken.
// ---------------------------------------------------------------------------
namespace topics {
inline constexpr const char* imu = "imu";
inline constexpr const char* range = "range";
inline constexpr const char* battery = "battery";
inline constexpr const char* pose = "pose";
inline constexpr const char* odom = "odom";
inline constexpr const char* cmd_vel = "cmd_vel";
inline constexpr const char* joints = "joints";
inline constexpr const char* tf = "tf";
}  // namespace topics

}  // namespace ls::msgs

// ---------------------------------------------------------------------------
// Message declarations.
//
// At global scope, because LOCKSTEP_MESSAGE opens `namespace ls` itself and
// cannot be invoked from inside ls::msgs. See the note at the top.
//
// Each one asserts that its type is relocatable, standard layout, trivially
// copyable, and that the field list covers every byte. The size asserts are
// the padding rule: no interior padding, so nothing has to be zero-filled
// before publish for a replay hash to reproduce.
// ---------------------------------------------------------------------------

LOCKSTEP_MESSAGE(ls::msgs::Vec3, x, y, z);
static_assert(sizeof(ls::msgs::Vec3) == 24, "Vec3 should be padding-free");
static_assert(!ls::message_traits<ls::msgs::Vec3>::has_padding, "");

LOCKSTEP_MESSAGE(ls::msgs::Quat, x, y, z, w);
static_assert(sizeof(ls::msgs::Quat) == 32, "Quat should be padding-free");
static_assert(!ls::message_traits<ls::msgs::Quat>::has_padding, "");

LOCKSTEP_MESSAGE(ls::msgs::Imu, stamp_ns, angular_velocity, linear_acceleration,
                 orientation, seq, valid);
static_assert(sizeof(ls::msgs::Imu) == 96, "Imu should be padding-free");
static_assert(!ls::message_traits<ls::msgs::Imu>::has_padding, "");

LOCKSTEP_MESSAGE(ls::msgs::Range, stamp_ns, distance_m, min_m, max_m, seq, valid, reserved);
static_assert(sizeof(ls::msgs::Range) == 32, "Range should be padding-free");
static_assert(!ls::message_traits<ls::msgs::Range>::has_padding, "");

LOCKSTEP_MESSAGE(ls::msgs::BatteryState, stamp_ns, voltage_v, current_a, charge_ah,
                 capacity_ah, percentage, temperature_c, seq, present);
static_assert(sizeof(ls::msgs::BatteryState) == 40, "BatteryState should be padding-free");
static_assert(!ls::message_traits<ls::msgs::BatteryState>::has_padding, "");

LOCKSTEP_MESSAGE(ls::msgs::Pose, stamp_ns, position, orientation, seq, reserved);
static_assert(sizeof(ls::msgs::Pose) == 72, "Pose should be padding-free");
static_assert(!ls::message_traits<ls::msgs::Pose>::has_padding, "");

LOCKSTEP_MESSAGE(ls::msgs::Twist, stamp_ns, linear, angular, seq, reserved);
static_assert(sizeof(ls::msgs::Twist) == 64, "Twist should be padding-free");
static_assert(!ls::message_traits<ls::msgs::Twist>::has_padding, "");

LOCKSTEP_MESSAGE(ls::msgs::Odometry, stamp_ns, position, orientation, linear_velocity,
                 angular_velocity, seq, reserved);
static_assert(sizeof(ls::msgs::Odometry) == 120, "Odometry should be padding-free");
static_assert(!ls::message_traits<ls::msgs::Odometry>::has_padding, "");

LOCKSTEP_MESSAGE(ls::msgs::JointState, stamp_ns, position, velocity, effort, count, seq);
static_assert(sizeof(ls::msgs::JointState) == 400, "JointState should be padding-free");
static_assert(!ls::message_traits<ls::msgs::JointState>::has_padding, "");

LOCKSTEP_MESSAGE(ls::msgs::Transform, stamp_ns, translation, rotation, parent, child, seq,
                 reserved);
static_assert(sizeof(ls::msgs::frame_id) == 32, "frame_id should pack to 32 bytes");
static_assert(sizeof(ls::msgs::Transform) == 136, "Transform should be padding-free");
static_assert(!ls::message_traits<ls::msgs::Transform>::has_padding, "");
