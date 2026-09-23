// One node of the five-node demo. The role decides what it publishes and what
// it subscribes to.
//
//   camera    -> cam/front              (DemoFrame, 64KB body)
//   imu       -> imu/raw                (DemoImu)
//   detector  cam/front -> perception/detections
//   fusion    perception/detections + imu/raw -> state/pose
//   logger    state/pose -> journal + stats on stderr
//
//   argv[1] segment name   argv[2] role   argv[3] milliseconds to run
//   argv[4] optional journal path (logger only)
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "demo_msgs.hpp"
#include "lockstep/core/process.hpp"
#include "lockstep/pubsub.hpp"
#include "lockstep/replay/journal.hpp"

namespace {

constexpr std::size_t kFrameBody = 64 * 1024;

bool expired(std::uint64_t start_ns, std::uint64_t run_ms) {
  return (ls::now_ns() - start_ns) / 1000000u >= run_ms;
}

int run_camera(ls::bus& b, std::uint64_t run_ms) {
  auto pub = ls::publisher<DemoFrame>::create(b, "cam/front", 32, kFrameBody);
  const std::uint64_t t0 = ls::now_ns();
  std::uint64_t seq = 0;

  while (!expired(t0, run_ms)) {
    auto ln = pub.loan_message();
    ln->stamp_ns = ls::now_ns();
    ln->seq = seq;
    ln->width = 640;
    ln->height = 480;
    ln->frame_id.assign("front");
    // Touch the payload so the cost is a real frame's worth of work, not a
    // pointer swap. This is the 64KB that a copying transport would copy twice.
    std::memset(ln.body(), static_cast<int>(seq & 0xff), kFrameBody);
    ln->pixels.bind(ln.body(), kFrameBody);
    ln.set_body_size(kFrameBody);
    pub.publish(ln);
    ++seq;
    ls::sleep_us(10000);  // ~100 Hz
  }
  std::fprintf(stderr, "  [camera]   published %llu frames\n",
               (unsigned long long)seq);
  return 0;
}

int run_imu(ls::bus& b, std::uint64_t run_ms) {
  auto pub = ls::publisher<DemoImu>::create(b, "imu/raw", 256);
  const std::uint64_t t0 = ls::now_ns();
  std::uint64_t seq = 0;

  while (!expired(t0, run_ms)) {
    auto ln = pub.loan_message();
    ln->stamp_ns = ls::now_ns();
    ln->seq = seq;
    ln->ax = static_cast<float>(seq % 100) * 0.01f;
    ln->ay = 0.5f;
    ln->az = 9.81f;
    ln->gz = static_cast<float>((seq % 7)) - 3.0f;
    pub.publish(ln);
    ++seq;
    ls::sleep_us(2000);  // ~500 Hz
  }
  std::fprintf(stderr, "  [imu]      published %llu samples\n",
               (unsigned long long)seq);
  return 0;
}

int run_detector(ls::bus& b, std::uint64_t run_ms) {
  auto sub = ls::subscriber<DemoFrame>::attach(b, "cam/front");
  auto pub = ls::publisher<DemoDetection>::create(b, "perception/detections", 64);

  const std::uint64_t t0 = ls::now_ns();
  std::uint64_t seen = 0, emitted = 0, lost = 0;

  while (!expired(t0, run_ms)) {
    ls::sample<DemoFrame> s;
    const auto rr = sub.take(s);
    if (rr == ls::read_result::overrun) { ++lost; continue; }
    if (rr != ls::read_result::ok) { ls::sleep_us(500); continue; }
    ++seen;

    // "Detection": read some of the frame body, so the node genuinely consumes
    // the zero-copy payload rather than just its descriptor.
    std::uint32_t acc = 0;
    const auto* px = s->pixels.data();
    for (std::size_t i = 0; i < s->pixels.size(); i += 4096) acc += px[i];

    auto ln = pub.loan_message();
    ln->stamp_ns = ls::now_ns();
    ln->frame_seq = s->seq;
    ln->count = acc % 5;
    ln->confidence = 0.5f + static_cast<float>(acc % 50) * 0.01f;
    pub.publish(ln);
    ++emitted;
  }
  std::fprintf(stderr, "  [detector] frames %llu, detections %llu, overruns %llu\n",
               (unsigned long long)seen, (unsigned long long)emitted,
               (unsigned long long)lost);
  return 0;
}

int run_fusion(ls::bus& b, std::uint64_t run_ms) {
  auto det = ls::subscriber<DemoDetection>::attach(b, "perception/detections");
  auto imu = ls::subscriber<DemoImu>::attach(b, "imu/raw");
  auto pub = ls::publisher<DemoPose>::create(b, "state/pose", 64);

  const std::uint64_t t0 = ls::now_ns();
  std::uint64_t dets = 0, imus = 0, poses = 0;
  float yaw = 0.0f;

  while (!expired(t0, run_ms)) {
    bool did = false;

    ls::sample<DemoImu> is;
    while (imu.take(is) == ls::read_result::ok) {
      yaw += is->gz * 0.002f;
      ++imus;
      did = true;
    }

    ls::sample<DemoDetection> ds;
    while (det.take(ds) == ls::read_result::ok) {
      ++dets;
      auto ln = pub.loan_message();
      ln->stamp_ns = ls::now_ns();
      ln->detections_seen = dets;
      ln->x = static_cast<float>(dets) * 0.1f;
      ln->y = static_cast<float>(ds->count);
      ln->z = 0.0f;
      ln->yaw = yaw;
      pub.publish(ln);
      ++poses;
      did = true;
    }
    if (!did) ls::sleep_us(500);
  }
  std::fprintf(stderr, "  [fusion]   imu %llu, detections %llu, poses %llu\n",
               (unsigned long long)imus, (unsigned long long)dets,
               (unsigned long long)poses);
  return 0;
}

int run_logger(ls::bus& b, std::uint64_t run_ms, const char* journal_path) {
  auto sub = ls::subscriber<DemoPose>::attach(b, "state/pose");
  ls::journal j = ls::journal::create(journal_path, 16u << 20);
  ls::deterministic_clock clk(ls::run_mode::record, &j);
  ls::output_hasher out;

  const std::uint64_t t0 = ls::now_ns();
  std::uint64_t logged = 0;

  while (!expired(t0, run_ms)) {
    ls::sample<DemoPose> s;
    const auto rr = sub.take(s);
    if (rr != ls::read_result::ok) { ls::sleep_us(500); continue; }

    DemoPose copy{};
    if (!s.copy_out(copy)) continue;

    j.append(ls::record_kind::message, 0, s.sequence(), s.stamp_ns(), &copy,
             sizeof(copy));
    const std::uint64_t t = clk.now_ns();
    out.mix_value(copy.detections_seen);
    out.mix_value(copy.yaw);
    out.mix_value(t);
    ++logged;
  }

  std::fprintf(stderr,
               "  [logger]   poses %llu, journal records %llu, output hash %016llx\n",
               (unsigned long long)logged, (unsigned long long)j.records(),
               (unsigned long long)out.value());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::fprintf(stderr, "usage: demo_node <segment> <role> <run-ms> [journal]\n");
    return 2;
  }
  const std::string segment = argv[1];
  const std::string role = argv[2];
  const auto run_ms = std::strtoull(argv[3], nullptr, 10);

  try {
    ls::bus b = ls::bus::open(segment);
    if (role == "camera") return run_camera(b, run_ms);
    if (role == "imu") return run_imu(b, run_ms);
    if (role == "detector") return run_detector(b, run_ms);
    if (role == "fusion") return run_fusion(b, run_ms);
    if (role == "logger") {
      const std::string journal =
          argc > 4 ? std::string(argv[4]) : ls::temp_path("lockstep-demo.jrnl");
      return run_logger(b, run_ms, journal.c_str());
    }
    std::fprintf(stderr, "unknown role: %s\n", role.c_str());
    return 2;
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "  [%s] failed: %s\n", role.c_str(), ex.what());
    return 1;
  }
}
