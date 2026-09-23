// The second process in the phase 2 acceptance test.
//
// Launched by test_two_process via fork+exec, so it shares nothing with the
// parent except the named segment: a fresh address space, a fresh loader, its
// own ASLR base. Everything it knows about the bus it reads out of the mapped
// bytes.
//
//   argv[1]  segment name
//   argv[2]  the arena offset the parent claims to have written the frame at
//
// Exits 0 if every check passes, 1 otherwise.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string_view>

#include "lockstep/core/process.hpp"
#include "lockstep/shm/bus.hpp"
#include "support/demo_msgs.hpp"

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::fprintf(stderr, "  [child] FAIL %s\n", what);
    ++failures;
  }
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: two_process_child <segment-name> <frame-offset>\n");
    return 2;
  }
  const char* name = argv[1];
  const auto expected_offset = std::strtoull(argv[2], nullptr, 10);

  try {
    ls::bus b = ls::bus::open(name);
    check(b.valid(), "bus::open returned a valid bus");

    // 1. The handshake. Attaching with the type the publisher announced works.
    ls::topic_entry* e = nullptr;
    const auto st = b.topics().attach<CameraFrame>("cam/front", &e);
    check(st == ls::attach_status::ok, "attach<CameraFrame> succeeded");
    if (st != ls::attach_status::ok) {
      std::fprintf(stderr, "  [child] attach said: %s\n", ls::to_string(st));
      return 1;
    }

    // 2. The handshake refuses a stale header. Same size, two fields swapped.
    ls::topic_entry* bad = nullptr;
    const auto bad_st = b.topics().attach<CameraFrameReordered>("cam/front", &bad);
    check(bad_st == ls::attach_status::layout_mismatch,
          "attach<CameraFrameReordered> was refused as a layout mismatch");

    // 3. The publisher recorded where it put the frame. (Phase 4 replaces this
    //    field with a real ring; for phase 2 it is just a slot pointer.)
    const std::uint64_t off = e->ring_offset.load(std::memory_order_acquire);
    check(off != 0, "publisher recorded a frame offset");
    check(off == expected_offset, "frame offset matches what the parent reported");

    const auto* frame = b.seg().at<CameraFrame>(off);
    check(b.seg().contains(frame, sizeof(CameraFrame)), "frame lies inside the segment");

    // 4. Scalar fields survived the crossing.
    check(frame->stamp_ns == 1234567890123ull, "stamp_ns");
    check(frame->width == 640u, "width");
    check(frame->height == 480u, "height");
    check(frame->frame_id == std::string_view("front_camera"), "frame_id");

    // 5. The part that phase 1 exists for. `pixels` is an offset_ptr into a
    //    DIFFERENT arena block. The segment is mapped at a different base here
    //    than in the parent, so a raw pointer would be garbage; the self-
    //    relative offset has to resolve against this process's own mapping.
    check(frame->pixels.size() == 4096u, "pixels span length");
    const auto* px = frame->pixels.data();
    check(px != nullptr, "pixels resolved to non-null");
    check(b.seg().contains(px, 4096), "pixels point inside THIS process's mapping");

    bool content_ok = true;
    for (std::size_t i = 0; i < 4096; ++i) {
      if (px[i] != static_cast<std::uint8_t>((i * 7) & 0xff)) {
        content_ok = false;
        break;
      }
    }
    check(content_ok, "pixel payload matches the pattern the parent wrote");

    // 6. And the resolved address must be the parent's arena offset, recomputed
    //    against this process's base rather than copied.
    const std::uint64_t px_off = b.seg().offset_of(px);
    check(px_off == frame->pixels.data_.raw_offset() +
                        b.seg().offset_of(&frame->pixels),
          "pixel offset is self-relative, not absolute");

    std::fprintf(stderr, "  [child] base=%p pid=%d checks_failed=%d\n",
                 b.seg().base(), static_cast<int>(ls::self_pid()), failures);
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "  [child] threw: %s\n", ex.what());
    return 1;
  }

  return failures == 0 ? 0 : 1;
}
