// Message types for the five-node demo.
#pragma once

#include <cstdint>

#include "lockstep/containers/inline_string.hpp"
#include "lockstep/containers/shm_span.hpp"
#include "lockstep/core/message.hpp"

struct DemoImu {
  std::uint64_t stamp_ns;
  std::uint64_t seq;
  float ax;
  float ay;
  float az;
  float gz;
};
LOCKSTEP_MESSAGE(DemoImu, stamp_ns, seq, ax, ay, az, gz);

struct DemoFrame {
  std::uint64_t stamp_ns;
  std::uint64_t seq;
  std::uint32_t width;
  std::uint32_t height;
  ls::inline_string<16> frame_id;
  ls::shm_span<std::uint8_t> pixels;
};
LOCKSTEP_MESSAGE(DemoFrame, stamp_ns, seq, width, height, frame_id, pixels);

struct DemoDetection {
  std::uint64_t stamp_ns;
  std::uint64_t frame_seq;
  std::uint32_t count;
  float confidence;
};
LOCKSTEP_MESSAGE(DemoDetection, stamp_ns, frame_seq, count, confidence);

struct DemoPose {
  std::uint64_t stamp_ns;
  std::uint64_t detections_seen;
  float x;
  float y;
  float z;
  float yaw;
};
LOCKSTEP_MESSAGE(DemoPose, stamp_ns, detections_seen, x, y, z, yaw);
