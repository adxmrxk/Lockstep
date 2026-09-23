// Phase 2's acceptance test: "a segment two processes can map".
//
// The parent lays out a bus, announces a topic, writes a CameraFrame into one
// arena block with its pixel payload in another, then fork+execs a separate
// binary. The child shares no state with this process beyond the segment name
// on its command line -- different address space, its own ASLR base -- and has
// to recover everything from the mapped bytes.
//
// If offset_ptr, the layout hash, the registry or the arena were wrong, this is
// the test that notices.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include "lockstep/core/process.hpp"
#include "lockstep/shm/bus.hpp"
#include "support/check.hpp"
#include "support/process.hpp"
#include "support/demo_msgs.hpp"

#ifndef LOCKSTEP_CHILD_BINARY
#error "LOCKSTEP_CHILD_BINARY must be defined by the build"
#endif

namespace {

constexpr std::size_t kPixelBytes = 4096;

int run_child(const std::string& segment_name, std::uint64_t frame_offset) {
  ls::test::child_process c =
      ls::test::spawn(LOCKSTEP_CHILD_BINARY,
                      {segment_name, std::to_string(frame_offset)});
  if (!c.valid()) return -1;
  return ls::test::wait_for(c);
}

void a_second_process_reads_what_this_one_published() {
  const std::string name = "lockstep-2proc-" + std::to_string(ls::self_pid());
  ls::segment::unlink(name);

  ls::bus b = ls::bus::create(name, {{64, 16}, {4096, 8}});

  // Announce the topic. This is what the child will handshake against.
  ls::topic_entry* topic = nullptr;
  LS_CHECK(b.topics().announce<CameraFrame>("cam/front", &topic) ==
           ls::attach_status::ok);
  topic->publisher_pid.store(ls::self_pid(),
                             std::memory_order_relaxed);

  // Payload block: the pixels live out in the arena, not in the message.
  const std::uint64_t pixel_off = b.allocator().allocate(kPixelBytes);
  LS_CHECK(pixel_off != 0);
  auto* pixels = b.seg().at<std::uint8_t>(pixel_off);
  for (std::size_t i = 0; i < kPixelBytes; ++i)
    pixels[i] = static_cast<std::uint8_t>((i * 7) & 0xff);

  // Message block.
  const std::uint64_t frame_off = b.allocator().allocate(sizeof(CameraFrame));
  LS_CHECK(frame_off != 0);
  LS_CHECK(frame_off != pixel_off);
  auto* frame = b.seg().at<CameraFrame>(frame_off);

  // CameraFrame has padding, so the slot has to be zeroed before it is filled.
  // Indeterminate padding bytes would make a replay hash over the slot
  // irreproducible -- this is the rule message_traits::has_padding exists to
  // announce, and phase 2 is the first place it is actually obeyed.
  LS_CHECK(ls::message_traits<CameraFrame>::has_padding);
  std::memset(static_cast<void*>(frame), 0, sizeof(CameraFrame));

  frame->stamp_ns = 1234567890123ull;
  frame->width = 640;
  frame->height = 480;
  LS_CHECK(frame->frame_id.assign("front_camera"));
  frame->pixels.bind(pixels, kPixelBytes);

  // Sanity: it reads back correctly in the process that wrote it.
  LS_CHECK_EQ(frame->pixels.size(), kPixelBytes);
  LS_CHECK_EQ(frame->pixels[9], static_cast<std::uint8_t>((9 * 7) & 0xff));

  // Hand the child the slot. Phase 4 replaces this with the ring.
  topic->ring_offset.store(frame_off, std::memory_order_release);

  std::fprintf(stderr, "  [parent] base=%p pid=%d frame_off=%llu\n", b.seg().base(),
               static_cast<int>(ls::self_pid()),
               static_cast<unsigned long long>(frame_off));

  const int rc = run_child(name, frame_off);
  LS_CHECK_EQ(rc, 0);

  // The child mapped, read and exited; the parent's view must be untouched.
  LS_CHECK_EQ(frame->width, 640u);
  LS_CHECK_EQ(frame->pixels.size(), kPixelBytes);
  LS_CHECK(frame->frame_id == std::string_view("front_camera"));
}

// A process attaching to a bus that does not exist must fail cleanly rather
// than mapping something arbitrary.
void opening_a_missing_bus_fails() {
  bool threw = false;
  try {
    ls::bus gone = ls::bus::open("lockstep-does-not-exist-" + std::to_string(ls::self_pid()));
  } catch (const std::exception&) {
    threw = true;
  }
  LS_CHECK(threw);
}

}  // namespace

int main() {
  a_second_process_reads_what_this_one_published();
  opening_a_missing_bus_fails();
  return ls::test::summary("two_process");
}
