// The five-node demo: a perception-to-control graph, a crash in the middle of
// it, and a replay of what came out.
//
//   camera  --cam/front-->  detector  --detections-->  fusion  --pose-->  logger
//   imu     --imu/raw---------------------------------^
//
// Five real processes on one shared-memory segment. Part way through, the
// detector is SIGKILLed. The reaper reclaims its topic, a replacement detector
// starts, and the graph carries on -- which is the whole crash-consistency
// story reduced to something you can watch happen.
//
// Afterwards the logger's journal is replayed and its output hash recompared,
// which is the replay story in the same terms.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include "demo_msgs.hpp"
#include "lockstep/pubsub.hpp"
#include "lockstep/replay/journal.hpp"
#include "lockstep/shm/liveness.hpp"

#ifndef LOCKSTEP_DEMO_NODE_BINARY
#error "LOCKSTEP_DEMO_NODE_BINARY must be defined by the build"
#endif

namespace {

pid_t spawn(const std::string& segment, const char* role, std::uint64_t run_ms,
            const char* journal = nullptr) {
  const std::string ms = std::to_string(run_ms);
  const pid_t pid = ::fork();
  if (pid == 0) {
    if (journal != nullptr) {
      ::execl(LOCKSTEP_DEMO_NODE_BINARY, LOCKSTEP_DEMO_NODE_BINARY, segment.c_str(),
              role, ms.c_str(), journal, static_cast<char*>(nullptr));
    } else {
      ::execl(LOCKSTEP_DEMO_NODE_BINARY, LOCKSTEP_DEMO_NODE_BINARY, segment.c_str(),
              role, ms.c_str(), static_cast<char*>(nullptr));
    }
    std::fprintf(stderr, "exec failed for %s: %s\n", role, std::strerror(errno));
    ::_exit(127);
  }
  return pid;
}

std::uint64_t replay_logger_journal(const std::string& path) {
  ls::journal j = ls::journal::open_read(path);
  ls::deterministic_clock clk(ls::run_mode::replay, &j);
  ls::output_hasher out;

  ls::record r;
  while (j.next(r)) {
    if (r.kind != ls::record_kind::message) continue;
    DemoPose p{};
    std::memcpy(&p, r.payload, sizeof(p));
    const std::uint64_t t = clk.now_ns();
    out.mix_value(p.detections_seen);
    out.mix_value(p.yaw);
    out.mix_value(t);
  }
  return out.value();
}

}  // namespace

int main() {
  const std::string segment = "lockstep-demo-" + std::to_string(::getpid());
  const std::string journal = "/tmp/lockstep-demo-" + std::to_string(::getpid()) + ".jrnl";
  constexpr std::uint64_t kRunMs = 3000;

  ls::segment::unlink(segment);

  std::printf("Lockstep five-node demo\n");
  std::printf("=======================\n\n");
  std::printf("  camera --cam/front--> detector --detections--> fusion --pose--> logger\n");
  std::printf("  imu    --imu/raw------------------------------^\n\n");

  // The launcher owns the segment, so it outlives every node.
  ls::bus b = ls::bus::create(segment, {{64, 4096},
                                        {256, 1024},
                                        {4096, 256},
                                        {65536, 64},
                                        {1u << 20, 16}});
  std::printf("  segment %s, %llu bytes\n\n", segment.c_str(),
              (unsigned long long)b.seg().size());

  std::printf("-- starting nodes --\n");
  std::vector<std::pair<std::string, pid_t>> nodes;
  nodes.emplace_back("camera", spawn(segment, "camera", kRunMs));
  nodes.emplace_back("imu", spawn(segment, "imu", kRunMs));
  ::usleep(150000);  // let the publishers create their topics first
  pid_t detector = spawn(segment, "detector", kRunMs);
  nodes.emplace_back("detector", detector);
  ::usleep(150000);
  nodes.emplace_back("fusion", spawn(segment, "fusion", kRunMs));
  ::usleep(150000);
  nodes.emplace_back("logger", spawn(segment, "logger", kRunMs, journal.c_str()));

  for (const auto& n : nodes)
    std::printf("     %-9s pid %d\n", n.first.c_str(), (int)n.second);

  // ---------------------------------------------------------------- crash --
  ::usleep(1000000);
  std::printf("\n-- killing the detector with SIGKILL --\n");
  ::kill(detector, SIGKILL);
  int st = 0;
  ::waitpid(detector, &st, 0);
  std::printf("     detector pid %d killed (signalled=%d)\n", (int)detector,
              (int)WIFSIGNALED(st));

  ls::topic_entry* det_topic = b.topics().find("perception/detections");
  std::printf("     topic publisher_pid before reap: %u\n",
              det_topic ? det_topic->publisher_pid.load() : 0u);

  ls::reaper rp(b);
  const ls::reap_report rep = rp.scan();
  std::printf("     reaper: %u topics scanned, %u publishers reclaimed, "
              "%u slots healed\n",
              rep.topics_scanned, rep.publishers_reclaimed, rep.slots_healed);
  std::printf("     topic publisher_pid after reap:  %u\n",
              det_topic ? det_topic->publisher_pid.load() : 0u);

  // ------------------------------------------------------------- recovery --
  std::printf("\n-- starting a replacement detector --\n");
  const pid_t detector2 = spawn(segment, "detector", kRunMs - 1200);
  std::printf("     detector pid %d took the topic over\n", (int)detector2);

  // --------------------------------------------------------------- finish --
  int failures = 0;
  for (auto& n : nodes) {
    if (n.second == detector) continue;
    int status = 0;
    ::waitpid(n.second, &status, 0);
    if (WIFEXITED(status) && WEXITSTATUS(status) != 0) ++failures;
  }
  int status2 = 0;
  ::waitpid(detector2, &status2, 0);

  std::printf("\n-- node summaries (stderr above) --\n");

  // ---------------------------------------------------------------- replay --
  std::printf("\n-- replaying the logger's journal --\n");
  try {
    ls::journal j = ls::journal::open_read(journal);
    std::printf("     journal %llu records, %llu bytes, content hash %016llx\n",
                (unsigned long long)j.records(), (unsigned long long)j.used(),
                (unsigned long long)j.content_hash());
    j.close();

    const std::uint64_t a = replay_logger_journal(journal);
    const std::uint64_t c = replay_logger_journal(journal);
    std::printf("     replay #1 output hash %016llx\n", (unsigned long long)a);
    std::printf("     replay #2 output hash %016llx\n", (unsigned long long)c);
    std::printf("     %s\n", a == c ? "identical -- replay is deterministic"
                                    : "DIVERGED -- replay is not deterministic");
    if (a != c) ++failures;
  } catch (const std::exception& ex) {
    std::printf("     replay failed: %s\n", ex.what());
    ++failures;
  }

  ::unlink(journal.c_str());
  std::printf("\n%s\n", failures == 0 ? "demo completed with no failures"
                                      : "demo completed WITH FAILURES");
  return failures == 0 ? 0 : 1;
}
