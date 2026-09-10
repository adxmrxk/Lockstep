// Message types shared between the phase 2 tests and the two-process child.
// Both binaries must build these from the same header, which is the point: the
// layout hash they compute is what the registry handshake compares.
#pragma once

#include <cstdint>

#include "lockstep/containers/inline_string.hpp"
#include "lockstep/containers/shm_span.hpp"
#include "lockstep/core/message.hpp"

struct ImuSample {
  std::uint64_t stamp_ns;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};
LOCKSTEP_MESSAGE(ImuSample, stamp_ns, ax, ay, az, gx, gy, gz);

struct CameraFrame {
  std::uint64_t stamp_ns;
  std::uint32_t width;
  std::uint32_t height;
  ls::inline_string<16> frame_id;
  ls::shm_span<std::uint8_t> pixels;
};
LOCKSTEP_MESSAGE(CameraFrame, stamp_ns, width, height, frame_id, pixels);

// Same field names and same size as CameraFrame, but width and height are
// swapped in declaration order. A node built against this header must be
// refused by a bus publishing the real CameraFrame -- that refusal is the whole
// reason the layout hash exists.
struct CameraFrameReordered {
  std::uint64_t stamp_ns;
  std::uint32_t height;
  std::uint32_t width;
  ls::inline_string<16> frame_id;
  ls::shm_span<std::uint8_t> pixels;
};
LOCKSTEP_MESSAGE(CameraFrameReordered, stamp_ns, height, width, frame_id, pixels);
