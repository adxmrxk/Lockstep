// robot/elegoo/robot_msgs.hpp : the topics the robot bridge speaks.
//
// Three messages, deliberately few. The bus is the interesting part of this
// project; the robot is the body attached to it, and a body needs almost no
// vocabulary: what do you see, where should I go, are you still there.
//
// Every one of these is laid out so sizeof() lands on a multiple of 8 with no
// interior padding -- check the static_asserts below. That is not tidiness.
// A replay hash is taken over the whole message slot, so padding bytes, being
// indeterminate, would make a recorded run irreproducible unless the publisher
// zeroed them first. Designing the padding away removes the requirement instead
// of relying on remembering it.
#pragma once

#include <cstdint>

#include "lockstep/core/message.hpp"

// What the ultrasonic sensor reported.
//
// `valid` matters more than it looks. The HC-SR04 misreads constantly -- it
// returns 0 or an absurd number when the echo is lost off an angled surface --
// and a controller that steers on those will jerk for no reason. The bridge
// decides validity once, here, so every subscriber downstream agrees about it
// rather than each inventing its own filter.
struct RobotRange {
  std::uint64_t stamp_ns;
  float distance_cm;
  std::uint32_t seq;
  std::uint32_t valid;     // 0 = implausible reading, ignore distance_cm
  std::uint32_t reserved;
};
LOCKSTEP_MESSAGE(RobotRange, stamp_ns, distance_cm, seq, valid, reserved);
static_assert(sizeof(RobotRange) == 24, "RobotRange should be padding-free");
static_assert(!ls::message_traits<RobotRange>::has_padding,
              "padding would have to be zero-filled for replay to reproduce");

// Where to drive. Differential: signed speeds per side, negative is reverse.
//
// The bridge is the only thing that knows the firmware's speed units and
// deadband, so this stays in the same 0..255-ish scale the hardware uses rather
// than inventing metres per second the bridge would only have to undo.
struct RobotDrive {
  std::uint64_t stamp_ns;
  std::int32_t left;
  std::int32_t right;
  std::uint32_t seq;
  std::uint32_t reserved;
};
LOCKSTEP_MESSAGE(RobotDrive, stamp_ns, left, right, seq, reserved);
static_assert(sizeof(RobotDrive) == 24, "RobotDrive should be padding-free");
static_assert(!ls::message_traits<RobotDrive>::has_padding, "");

// How the link to the robot is doing. Published whether or not the robot is
// reachable, so a subscriber can tell "no obstacles" apart from "no robot" --
// which are the same silence on the range topic and very different things to a
// controller.
struct RobotStatus {
  std::uint64_t stamp_ns;
  std::uint64_t commands_sent;
  std::uint64_t replies_ok;
  std::uint64_t errors;
  std::uint32_t connected;   // 0 = link down (or simulated)
  std::uint32_t seq;
};
LOCKSTEP_MESSAGE(RobotStatus, stamp_ns, commands_sent, replies_ok, errors, connected,
                 seq);
static_assert(sizeof(RobotStatus) == 40, "RobotStatus should be padding-free");
static_assert(!ls::message_traits<RobotStatus>::has_padding, "");

// Topic names, in one place so the bridge and every node agree by construction
// rather than by both spelling the same string correctly.
namespace robot_topics {
inline constexpr const char* range = "robot/range";
inline constexpr const char* drive = "robot/drive";
inline constexpr const char* status = "robot/status";
}  // namespace robot_topics
