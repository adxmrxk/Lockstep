// avoider : drive forward, back off when something is close.
//
// This is the node that shows why the bus is shaped the way it is. It contains
// no networking, no JSON, no notion that an Elegoo car exists. It subscribes to
// a distance and publishes a pair of motor speeds. Swap the bridge for a
// different robot and this file does not change.
//
// It also demonstrates the actual point of the project, on real hardware data:
//
//   --journal <path>   record the session: every range reading as delivered,
//                      every clock read, every decision
//   --replay <path>    re-run those decisions from the journal and print the
//                      output hash
//
// Record a drive, then replay it twice. The hashes match, bit for bit, because
// the journal reproduces not just what the robot SAW but when it saw it and
// what the clock said at the time. That is the thing a ROS 2 bag cannot do, and
// the reason a timing-dependent bug that happens once on the carpet can be made
// to happen again, on demand, at a desk.
//
// Every live run also drops a readable transcript in runs/run_<n>.txt as it
// goes. The journal is the authoritative record and the only thing a replay
// reads; the transcript is there so that after a drive goes wrong you can open
// a file and see what happened without running anything.
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <memory>
#include <string>

#include "avi.hpp"
#include "lockstep/core/process.hpp"
#include "lockstep/pubsub.hpp"
#include "lockstep/replay/journal.hpp"
#include "robot_msgs.hpp"

namespace {

// Ctrl-C has to be survivable, not fatal.
//
// Killed outright, a run loses the end of its video (the frame count and index
// are written at close) and never takes its snapshot. Catching the signal and
// leaving the loop normally means both still happen, which matters most in
// exactly the case you are most likely to hit Ctrl-C: something went wrong and
// you want the recording.
std::atomic<bool> g_stop{false};
extern "C" void on_signal(int) { g_stop.store(true, std::memory_order_relaxed); }

// Tuning. Deliberately crude -- the interesting part is reproducibility, not
// the sophistication of the controller.
constexpr float kStopDistanceCm = 25.0f;   // closer than this: retreat
constexpr float kSlowDistanceCm = 50.0f;   // closer than this: ease off
constexpr int kCruiseSpeed = 150;
constexpr int kSlowSpeed = 110;
constexpr int kTurnSpeed = 130;

// THE decision. A pure function of the reading and the clock, which is exactly
// what makes it replayable: give it the same inputs and it must produce the
// same output, every time, forever.
//
// `turn_phase` is deliberately derived from the clock rather than from a
// counter. That is not arbitrary -- it is the kind of hidden timing dependency
// that makes real robot bugs irreproducible, and having one here means the
// replay test is proving something instead of restating that pure functions are
// pure.
RobotDrive decide(const RobotRange& r, std::uint64_t now_ns, std::uint32_t seq) {
  RobotDrive d{};
  d.stamp_ns = now_ns;
  d.seq = seq;
  d.reserved = 0;

  if (r.valid == 0) {
    // No usable reading. Creep rather than stop dead: the sensor misreads
    // often enough that halting on every bad echo would make the car stutter
    // across the floor.
    d.left = kSlowSpeed;
    d.right = kSlowSpeed;
    return d;
  }

  if (r.distance_cm < kStopDistanceCm) {
    // Back and turn. Which way depends on the clock, so two runs that see the
    // same obstacle at different moments will turn differently -- and a replay
    // has to reproduce the clock to reproduce the turn.
    const bool turn_left = ((now_ns / 1000000ull) % 2ull) == 0ull;
    d.left = turn_left ? -kTurnSpeed : kTurnSpeed;
    d.right = turn_left ? kTurnSpeed : -kTurnSpeed;
    return d;
  }

  const int speed = r.distance_cm < kSlowDistanceCm ? kSlowSpeed : kCruiseSpeed;
  d.left = speed;
  d.right = speed;
  return d;
}

struct options {
  std::string bus = "lockstep-robot";
  std::string journal;
  std::string replay;
  std::string runs_dir = "runs";
  long duration_ms = 0;
  // 0 means measure the real rate from the frames rather than assert one.
  int video_fps = 0;
  int journal_mb = 128;
  // Flight recorder: keep a rolling window instead of filling up and stopping,
  // and write a snapshot of it when something goes wrong. For a robot that runs
  // for hours before misbehaving, this is the difference between a recording
  // you can use and a journal that stopped two minutes in.
  bool ring = false;
  // Consecutive blind sensor reads that trip a snapshot. 0 turns it off.
  int blind_trigger = 8;
  bool trace = false;
  bool write_log = true;
  bool write_video = true;
};

// Journal callback ids. A journalled record says which stream it came from, and
// the avoider records two: sensor readings, which drive the decisions and are
// replayed, and camera frames, which are replayed into a video and touch no
// decision at all.
//
// ORDERING, AND IT MATTERS: a frame must be appended AFTER the clock read that
// its decision made. In replay, deterministic_clock::now_ns() walks forward
// through the journal with the same cursor this file reads with, discarding
// whatever it passes until it finds a clock record. A frame written between the
// range reading and the clock read would be swallowed by that scan and never
// reach the video. See journal.hpp's replay branch.
constexpr std::uint32_t kRangeCallback = 0;
constexpr std::uint32_t kFrameCallback = 1;

// Why decide() returned what it did, in words.
//
// Derived from the same two values decide() branches on rather than reported by
// decide() itself, so the label cannot drift away from the behaviour it claims
// to describe without the thresholds above changing too.
const char* reason_for(const RobotRange& r) {
  if (r.valid == 0) return "sensor gave nothing";
  if (r.distance_cm < kStopDistanceCm) return "too close, backing off";
  if (r.distance_cm < kSlowDistanceCm) return "closing in, slowing down";
  return "clear road ahead";
}

// What the motor pair actually amounts to, so the table can be read without
// decoding speed numbers in your head.
const char* action_for(const RobotDrive& d) {
  if (d.left == 0 && d.right == 0) return "stopped";
  if (d.left > 0 && d.right < 0) return "spin right";
  if (d.left < 0 && d.right > 0) return "spin left";
  if (d.left < 0 && d.right < 0) return "reverse";
  if (d.left != d.right) return "curve";
  return d.left >= kCruiseSpeed ? "forward, fast" : "forward, slow";
}

// Local time, formatted by strftime. Used for the header of a run file
// ("%Y-%m-%d %H:%M:%S") and for naming a video ("%Y-%m-%d").
std::string local_time_as(const char* format) {
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
#if defined(_WIN32)
  if (localtime_s(&tm, &t) != 0) return "unknown";
#else
  if (localtime_r(&t, &tm) == nullptr) return "unknown";
#endif
  char buf[64];
  if (std::strftime(buf, sizeof buf, format, &tm) == 0) return "unknown";
  return buf;
}

std::string local_timestamp() { return local_time_as("%Y-%m-%d %H:%M:%S"); }
std::string local_datestamp() { return local_time_as("%Y-%m-%d"); }

constexpr int kRuleWidth = 78;

void write_rule(std::FILE* f, char c) {
  std::fprintf(f, "  ");
  for (int i = 0; i < kRuleWidth; ++i) std::fputc(c, f);
  std::fputc('\n', f);
}

void write_banner(std::FILE* f, const std::string& title,
                  const std::string& what, const std::string& journal) {
  std::fputc('\n', f);
  write_rule(f, '=');
  std::fprintf(f, "  %s\n", title.c_str());
  write_rule(f, '=');
  std::fprintf(f, "  when      %s\n", local_timestamp().c_str());
  std::fprintf(f, "  what      %s\n", what.c_str());
  std::fprintf(f, "  journal   %s\n",
               journal.empty() ? "none (this run cannot be replayed)"
                               : journal.c_str());
}

// One decision, one line. Shared by the live run and by --replay, so a run file
// and a replay of the same journal read identically.
void write_table_header(std::FILE* f) {
  std::fputc('\n', f);
  std::fprintf(f, "  %4s  %8s  %10s   %-14s  %9s   %s\n", "#", "TIME",
               "SENSOR", "WHAT IT DID", "SPEED L/R", "WHY");
  write_rule(f, '-');
}

void write_table_row(std::FILE* f, std::uint64_t index, double elapsed_s,
                     const RobotRange& r, const RobotDrive& d,
                     const char* note) {
  char sensor[16];
  if (r.valid != 0) {
    std::snprintf(sensor, sizeof sensor, "%.0f cm",
                  static_cast<double>(r.distance_cm));
  } else {
    std::snprintf(sensor, sizeof sensor, "no reading");
  }
  char speeds[16];
  std::snprintf(speeds, sizeof speeds, "%d/%d", d.left, d.right);

  std::fprintf(f, "  %4llu  %7.2fs  %10s   %-14s  %9s   %s\n",
               static_cast<unsigned long long>(index), elapsed_s, sensor,
               action_for(d), speeds, reason_for(r));
  // Lines up under the SENSOR column, which is what the note is about.
  if (note != nullptr) std::fprintf(f, "  %21s^-- %s\n", "", note);
}

// Running tally, so the file can end with an account of the drive in words
// instead of leaving you to read 28 rows and work it out.
//
// Everything here is derived from the rows as they stream past; nothing looks
// ahead, so a run that is killed partway still has a usable tally written for
// the part that happened.
struct run_stats {
  std::uint64_t decisions = 0;
  std::uint64_t good_reads = 0;
  std::uint64_t retreats = 0;
  float closest_cm = 0.0f;
  double closest_at_s = 0.0;
  bool have_closest = false;
  double last_s = 0.0;

  std::uint64_t blind_streak = 0;
  double blind_streak_from_s = 0.0;
  std::uint64_t worst_blind = 0;
  double worst_blind_from_s = 0.0;
  double worst_blind_to_s = 0.0;

  // Call once per decision, in order. Returns an annotation for this row, or
  // nullptr: the first blind row after a good one is worth pointing at, because
  // that is where a drive usually starts going wrong.
  const char* add(double t_s, const RobotRange& r) {
    ++decisions;
    last_s = t_s;

    if (r.valid != 0) {
      ++good_reads;
      blind_streak = 0;
      if (!have_closest || r.distance_cm < closest_cm) {
        closest_cm = r.distance_cm;
        closest_at_s = t_s;
        have_closest = true;
      }
      if (r.distance_cm < kStopDistanceCm) ++retreats;
      return nullptr;
    }

    const bool streak_begins = blind_streak == 0;
    if (streak_begins) blind_streak_from_s = t_s;
    ++blind_streak;
    if (blind_streak >= worst_blind) {
      worst_blind = blind_streak;
      worst_blind_from_s = blind_streak_from_s;
      worst_blind_to_s = t_s;
    }
    return streak_begins && decisions > 1 ? "sensor went quiet here" : nullptr;
  }

  void write(std::FILE* f, std::uint64_t hash, const std::string& journal,
             const std::string& video_note,
             const std::string& snapshot_note = std::string()) const {
    const double blind_s = worst_blind_to_s - worst_blind_from_s;
    const double pct = decisions > 0
        ? 100.0 * static_cast<double>(good_reads) / static_cast<double>(decisions)
        : 0.0;

    std::fputc('\n', f);
    write_rule(f, '-');
    std::fprintf(f, "  WHAT HAPPENED\n");
    write_rule(f, '-');
    std::fprintf(f, "  ran for              %.1f seconds, %llu decisions\n",
                 last_s, static_cast<unsigned long long>(decisions));
    std::fprintf(f, "  sensor answered      %llu of %llu reads (%.0f percent)\n",
                 static_cast<unsigned long long>(good_reads),
                 static_cast<unsigned long long>(decisions), pct);
    if (have_closest) {
      std::fprintf(f, "  closest it came      %.0f cm, at %.1fs\n",
                   static_cast<double>(closest_cm), closest_at_s);
    } else {
      std::fprintf(f, "  closest it came      never got a reading at all\n");
    }
    std::fprintf(f, "  times it backed off  %llu\n",
                 static_cast<unsigned long long>(retreats));
    if (worst_blind > 0) {
      std::fprintf(f, "  longest blind spell  %.1f seconds, %.1fs to %.1fs%s\n",
                   blind_s, worst_blind_from_s, worst_blind_to_s,
                   blind_s >= 2.0 ? "   <-- THIS IS A PROBLEM" : "");
    } else {
      std::fprintf(f, "  longest blind spell  none, the sensor never dropped out\n");
    }
    if (!video_note.empty()) {
      std::fprintf(f, "  video                %s\n", video_note.c_str());
    }
    if (!snapshot_note.empty()) {
      std::fprintf(f, "  snapshot             %s\n", snapshot_note.c_str());
    }

    std::fprintf(f, "\n  output hash          %016llx\n",
                 static_cast<unsigned long long>(hash));
    std::fprintf(f,
                 "  Replay the journal and you get this same hash every time.\n"
                 "  A different hash means the code changed, not the robot.\n");
    if (!journal.empty()) {
      std::fprintf(f,
                   "\n  Re-run these exact decisions, car switched off,"
                   " nothing moves:\n"
                   "      avoider --replay %s\n",
                   journal.c_str());
    }
    write_rule(f, '=');
  }
};

// Where a run's output goes:
//
//   runs/logs/run_7.txt                the readable transcript
//   runs/videos/2026-10-07_run_7.avi   the drive, if the camera was on
//
// Two directories rather than one because they are read by different things: a
// text file you open in an editor and scan, a video you double click. Mixed
// together, the directory becomes unbrowsable after a dozen runs.
std::filesystem::path logs_dir(const std::string& root) {
  return std::filesystem::path(root) / "logs";
}
std::filesystem::path videos_dir(const std::string& root) {
  return std::filesystem::path(root) / "videos";
}
std::filesystem::path snapshots_dir(const std::string& root) {
  return std::filesystem::path(root) / "snapshots";
}

// The readable transcript: runs/logs/run_1.txt, run_2.txt, ... Written by every
// invocation, live drive and replay alike, so there is always a file to open.
//
// Written and flushed as the run happens rather than buffered to the end, so
// that a run which is killed, crashes or hangs still leaves behind everything
// up to the moment it stopped. That is the case the file exists for.
class run_log {
 public:
  ~run_log() { if (f_ != nullptr) std::fclose(f_); }

  run_log() = default;
  run_log(const run_log&) = delete;
  run_log& operator=(const run_log&) = delete;

  // Returns false and leaves the log closed if the directory or file cannot be
  // created. A run must never fail because its transcript could not be opened.
  // `what` describes the invocation, e.g. "live drive on bus lockstep-robot"
  // or "replay of drive1.jrnl".
  bool open(const std::string& root, const std::string& what,
            const std::string& journal) {
    const std::filesystem::path dir = logs_dir(root);
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    if (ec) return false;

    number_ = next_run_number(dir.string());
    path_ = (dir / ("run_" + std::to_string(number_) + ".txt")).string();
#if defined(_WIN32)
    if (fopen_s(&f_, path_.c_str(), "w") != 0) f_ = nullptr;
#else
    f_ = std::fopen(path_.c_str(), "w");
#endif
    if (f_ == nullptr) { path_.clear(); return false; }

    write_banner(f_, "LOCKSTEP AVOIDER -- RUN " + std::to_string(number_),
                 what, journal);
    write_table_header(f_);
    std::fflush(f_);
    return true;
  }

  bool is_open() const { return f_ != nullptr; }
  const std::string& path() const { return path_; }
  unsigned number() const { return number_; }

  void row(std::uint64_t index, double elapsed_s, const RobotRange& r,
           const RobotDrive& d, const char* note) {
    if (f_ == nullptr) return;
    write_table_row(f_, index, elapsed_s, r, d, note);
    std::fflush(f_);
  }

  void finish(const run_stats& stats, std::uint64_t hash,
              const std::string& journal, const std::string& video_note,
              const std::string& snapshot_note = std::string()) {
    if (f_ == nullptr) return;
    stats.write(f_, hash, journal, video_note, snapshot_note);
    std::fflush(f_);
  }

 private:
  // Highest run_<n>.txt already present, plus one. Names that are not run_<n>
  // are ignored, so dropping other files in runs/ does not disturb the numbers.
  static unsigned next_run_number(const std::string& dir) {
    unsigned highest = 0;
    std::error_code ec;
    for (const auto& e : std::filesystem::directory_iterator(dir, ec)) {
      const std::string name = e.path().filename().string();
      if (name.rfind("run_", 0) != 0) continue;
      if (e.path().extension() != ".txt") continue;
      const std::string digits = name.substr(4, name.size() - 8);
      if (digits.empty()) continue;
      if (digits.find_first_not_of("0123456789") != std::string::npos) continue;
      const unsigned n = static_cast<unsigned>(std::strtoul(digits.c_str(),
                                                            nullptr, 10));
      if (n > highest) highest = n;
    }
    return highest + 1;
  }

  std::FILE* f_ = nullptr;
  std::string path_;
  unsigned number_ = 0;
};

// The video for one run, opened on the first frame that actually arrives.
//
// Lazy on purpose. Most runs have no camera -- the bridge needs --camera for
// that -- and eagerly creating the file would litter runs/videos with 224 byte
// headers containing nothing, each of which looks like a video and plays as an
// error.
class video_sink {
 public:
  // `fps` of 0 means "measure it from the frames", which is the default. The
  // camera is pulled on a best-effort loop over WiFi, so the rate it actually
  // achieved is never the rate it was asked for, and a video tagged with the
  // requested rate plays at the wrong speed.
  video_sink(std::string dir, unsigned run_number, int fps)
      : dir_(std::move(dir)), run_(run_number), fps_(fps) {}

  void add(const std::uint8_t* jpeg, std::uint32_t bytes, std::uint64_t stamp_ns) {
    if (!enabled_) return;
    if (!w_.is_open() && !open_now()) { enabled_ = false; return; }
    if (!w_.add_frame(jpeg, bytes)) return;
    if (stamp_ns == 0) return;              // no usable timestamp to measure with
    if (first_stamp_ == 0) first_stamp_ = stamp_ns;
    last_stamp_ = stamp_ns;
  }

  // Returns the "<path> (<n> frames)" line for the summary, or "" if no video
  // was written.
  std::string close() {
    if (!w_.is_open()) return {};
    const std::string p = w_.path();
    const std::size_t n = w_.frames();
    const std::uint64_t bad = w_.rejected();

    const int measured = measured_fps(n);
    w_.set_fps(fps_ > 0 ? fps_ : measured);
    w_.close();

    if (n == 0) {
      std::error_code ec;
      std::filesystem::remove(p, ec);  // nothing in it; do not leave it behind
      return {};
    }
    std::string line = p + "  (" + std::to_string(n) + " frames at " +
                       std::to_string(fps_ > 0 ? fps_ : measured) + " fps";
    if (fps_ <= 0) line += ", measured";
    line += ")";
    if (bad > 0) line += ", " + std::to_string(bad) + " unreadable and skipped";
    return line;
  }

 private:
  // Frames per second the capture actually achieved, from the span between the
  // first and last frame. n-1 intervals, not n: two frames a second apart are
  // 1 fps, not 2.
  int measured_fps(std::size_t n) const {
    if (n < 2 || last_stamp_ <= first_stamp_) return kFallbackFps;
    const double span_s =
        static_cast<double>(last_stamp_ - first_stamp_) / 1e9;
    if (span_s <= 0.0) return kFallbackFps;
    const double fps = static_cast<double>(n - 1) / span_s;
    if (fps < 1.0) return 1;
    if (fps > 120.0) return 120;
    return static_cast<int>(fps + 0.5);
  }

  bool open_now() {
    std::error_code ec;
    std::filesystem::create_directories(dir_, ec);
    if (ec) return false;
    const std::string name =
        local_datestamp() + "_run_" + std::to_string(run_) + ".avi";
    // A placeholder rate: close() patches in the real one.
    return w_.open((std::filesystem::path(dir_) / name).string(),
                   fps_ > 0 ? fps_ : kFallbackFps);
  }

  static constexpr int kFallbackFps = 4;

  ls::avi::mjpeg_writer w_;
  std::string dir_;
  unsigned run_ = 0;
  int fps_ = 0;
  std::uint64_t first_stamp_ = 0;
  std::uint64_t last_stamp_ = 0;
  bool enabled_ = true;
};

void usage() {
  std::printf(
      "avoider -- obstacle-avoidance node for the Lockstep robot bus\n\n"
      "  --bus <name>      segment to attach to (default lockstep-robot)\n"
      "  --journal <path>  record the session for later replay\n"
      "  --replay <path>   replay a recorded session and print its hash\n"
      "  --trace           with --replay, print every re-executed decision to\n"
      "                    the terminal as well as to the run file\n"
      "  --runs-dir <dir>  output root (default runs). Transcripts go in\n"
      "                    <dir>/logs, video in <dir>/videos\n"
      "  --no-log          do not write a run transcript\n"
      "  --no-video        do not write video even if camera frames arrive\n"
      "  --video-fps <n>   force the video's playback rate. Omitted, the real\n"
      "                    rate is measured from the frames themselves\n"
      "  --journal-mb <n>  journal size in MB (default 128; video needs room)\n"
      "  --ring            flight recorder: keep a rolling window of the last\n"
      "                    --journal-mb instead of stopping when full, and dump\n"
      "                    a replayable snapshot when something goes wrong\n"
      "  --blind-trigger <n>  snapshot after n blind sensor reads in a row\n"
      "                    (default 8, 0 disables)\n"
      "  --duration <ms>   run time; 0 means until the bridge stops\n");
}

// Replay: no bus, no robot, no clock. Every input comes from the journal.
//
// NOTHING MOVES during a replay. The car can be switched off, boxed, or on the
// other side of the country. This re-executes the DECISIONS on this machine
// from the recording; it does not re-drive the robot.
int run_replay(const options& opt) {
  const std::string& path = opt.replay;
  ls::journal j = ls::journal::open_read(path);
  ls::deterministic_clock clk(ls::run_mode::replay, &j);
  ls::output_hasher out;

  std::uint32_t seq = 0;
  std::uint64_t decisions = 0;
  std::uint64_t t0 = 0;
  bool have_t0 = false;
  ls::record rec;
  run_stats stats;

  // A replay gets its own transcript, numbered alongside the live ones, so that
  // "open the newest file in runs/logs" is always the right move whatever was
  // run.
  run_log log;
  if (opt.write_log) {
    if (!log.open(opt.runs_dir, "replay of " + path, path)) {
      std::fprintf(stderr, "  warning: cannot write a run log in '%s',"
                           " continuing without one\n", opt.runs_dir.c_str());
    }
  }

  // The journal holds the camera frames, so a replay rebuilds the video from
  // it. That is the part worth noticing: the recording is one self-contained
  // file, and everything you look at afterwards -- the table, the summary, the
  // video -- is derived from it rather than captured alongside it.
  video_sink video(videos_dir(opt.runs_dir).string(), log.number(),
                   opt.video_fps);

  if (opt.trace) {
    write_banner(stdout, "REPLAY", "replay of " + path, path);
    write_table_header(stdout);
  }

  while (j.next(rec)) {
    if (rec.kind != ls::record_kind::message) continue;

    // Camera frames are inputs that no decision reads, so they are replayed
    // straight into the video and skipped here. They cannot affect the hash,
    // which is why adding video does not weaken the reproducibility claim.
    if (rec.callback == kFrameCallback) {
      if (opt.write_video && rec.payload != nullptr) {
        // rec.value is the stamp the frame was published with, so a replayed
        // video is paced by when the frames were really captured, not by how
        // fast this machine happens to walk the journal.
        video.add(static_cast<const std::uint8_t*>(rec.payload), rec.bytes,
                  rec.value);
      }
      continue;
    }
    if (rec.bytes != sizeof(RobotRange)) continue;  // not a reading

    RobotRange r{};
    std::memcpy(&r, rec.payload, sizeof(r));

    // The clock read happens INSIDE decide() in the recorded run too, so the
    // journal's clock record lands in the same place in the stream. Hoisting it
    // out changes nothing: it is still exactly one read, in the same position.
    const std::uint64_t now = clk.now_ns();
    const RobotDrive d = decide(r, now, seq++);
    out.mix_value(d.left);
    out.mix_value(d.right);
    ++decisions;

    if (!have_t0) { t0 = now; have_t0 = true; }
    const double elapsed_s = static_cast<double>(now - t0) / 1e9;
    const char* note = stats.add(elapsed_s, r);
    if (opt.trace) write_table_row(stdout, decisions - 1, elapsed_s, r, d, note);
    log.row(decisions - 1, elapsed_s, r, d, note);
  }

  const std::string video_note = video.close();
  log.finish(stats, out.value(), path, video_note);
  stats.write(stdout, out.value(), path, video_note);
  if (log.is_open()) {
    std::printf("\n  Saved to %s\n", log.path().c_str());
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
    if (a == "--help" || a == "-h") { usage(); return 0; }
    else if (a == "--trace") { opt.trace = true; }
    else if (a == "--no-log") { opt.write_log = false; }
    else if (a == "--no-video") { opt.write_video = false; }
    else if (a == "--runs-dir") { const char* v = next(); if (!v) return 1;
                                  opt.runs_dir = v; }
    else if (a == "--video-fps") { const char* v = next(); if (!v) return 1;
                                   opt.video_fps = std::atoi(v);
                                   if (opt.video_fps < 1) opt.video_fps = 1; }
    else if (a == "--journal-mb") { const char* v = next(); if (!v) return 1;
                                    opt.journal_mb = std::atoi(v);
                                    if (opt.journal_mb < 1) opt.journal_mb = 1; }
    else if (a == "--ring") { opt.ring = true; }
    else if (a == "--blind-trigger") { const char* v = next(); if (!v) return 1;
                                       opt.blind_trigger = std::atoi(v);
                                       if (opt.blind_trigger < 0) opt.blind_trigger = 0; }
    else if (a == "--bus") { const char* v = next(); if (!v) return 1; opt.bus = v; }
    else if (a == "--journal") { const char* v = next(); if (!v) return 1; opt.journal = v; }
    else if (a == "--replay") { const char* v = next(); if (!v) return 1; opt.replay = v; }
    else if (a == "--duration") { const char* v = next(); if (!v) return 1;
                                  opt.duration_ms = std::atol(v); }
    else { std::fprintf(stderr, "unknown option: %s\n", a.c_str()); usage(); return 1; }
  }

  if (!opt.replay.empty()) return run_replay(opt);

  std::signal(SIGINT, on_signal);
#if defined(SIGTERM)
  std::signal(SIGTERM, on_signal);
#endif

  std::printf("avoider\n=======\n");

  ls::bus b;
  try {
    b = ls::bus::open(opt.bus);
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "  cannot attach to bus '%s': %s\n", opt.bus.c_str(), ex.what());
    std::fprintf(stderr, "  start elegoo_bridge first (it creates the bus).\n");
    return 1;
  }

  ls::subscriber<RobotRange> range_sub;
  try {
    range_sub = ls::subscriber<RobotRange>::attach(b, robot_topics::range);
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "  no %s topic yet: %s\n", robot_topics::range, ex.what());
    return 1;
  }
  auto drive_pub = ls::publisher<RobotDrive>::create(b, robot_topics::drive, 32);

  // The camera topic only exists when the bridge was started with --camera, so
  // this attaches if it is there and shrugs if it is not. Same lazy pattern the
  // bridge uses for the drive topic, and for the same reason: the two processes
  // are started by hand, in either order, and neither should require the other
  // to have got there first.
  ls::subscriber<RobotFrame> camera_sub;
  bool have_camera = false;
  try {
    camera_sub = ls::subscriber<RobotFrame>::attach(b, robot_topics::camera);
    have_camera = true;
    std::printf("  camera found on %s, recording video\n", robot_topics::camera);
  } catch (const std::exception&) {
    std::printf("  no camera on %s (start the bridge with --camera for video)\n",
                robot_topics::camera);
  }
  // 64 KB is too much to put on the stack every cycle, and it only needs to
  // exist once.
  auto frame_buf = std::make_unique<RobotFrame>();

  ls::journal jr;
  const bool recording = !opt.journal.empty();
  if (recording) {
    // Sized for video. Sensor readings are 24 bytes and would fit in a
    // rounding error, but a camera frame is tens of kilobytes and the capture
    // thread runs at 10-15 Hz, so a half-minute drive is a few hundred
    // megabytes short of nothing and comfortably past the old 32 MB.
    const std::size_t jbytes = static_cast<std::size_t>(opt.journal_mb) << 20;
    jr = opt.ring ? ls::journal::create_ring(opt.journal, jbytes)
                  : ls::journal::create(opt.journal, jbytes);
    std::printf("  recording to %s (%d MB, %s)\n", opt.journal.c_str(),
                opt.journal_mb,
                opt.ring ? "rolling window" : "fills once then stops");
  }
  ls::deterministic_clock clk(recording ? ls::run_mode::record : ls::run_mode::live,
                              recording ? &jr : nullptr);
  ls::output_hasher out;

  run_log log;
  if (opt.write_log) {
    if (!log.open(opt.runs_dir, "live drive on bus " + opt.bus, opt.journal)) {
      std::fprintf(stderr, "  warning: cannot write a run log in '%s',"
                           " continuing without one\n", opt.runs_dir.c_str());
    }
  }

  video_sink video(videos_dir(opt.runs_dir).string(), log.number(),
                   opt.video_fps);

  write_banner(stdout, "LOCKSTEP AVOIDER -- LIVE",
               "live drive on bus " + std::string(opt.bus), opt.journal);
  write_table_header(stdout);

  const std::uint64_t t0 = ls::now_ns();
  std::uint32_t seq = 0;
  std::uint64_t decisions = 0, stale = 0, journal_full = 0;
  std::uint64_t log_t0 = 0;
  bool have_log_t0 = false;
  run_stats stats;

  // Snapshot bookkeeping. A trigger fires at most once per run: the first time
  // something goes wrong is the interesting one, and a robot that stays broken
  // would otherwise write a snapshot on every cycle for the rest of the drive.
  bool snapshot_taken = false;
  std::string snapshot_note;
  const auto take_snapshot = [&](const char* reason) {
    if (snapshot_taken || !recording || !opt.ring) return;
    snapshot_taken = true;
    std::error_code ec;
    const std::filesystem::path dir = snapshots_dir(opt.runs_dir);
    std::filesystem::create_directories(dir, ec);
    if (ec) return;
    const std::string path =
        (dir / (local_datestamp() + "_run_" + std::to_string(log.number()) +
                "_" + reason + ".jrnl")).string();
    if (!jr.snapshot(path)) return;
    snapshot_note = path + "  (" + reason + ", " +
                    std::to_string(jr.records()) + " records";
    if (jr.dropped() > 0) {
      snapshot_note += ", " + std::to_string(jr.dropped()) + " older dropped";
    }
    snapshot_note += ")";
    std::printf("\n  >>> SNAPSHOT TAKEN: %s\n"
                "      replay it with: avoider --replay %s --trace\n\n",
                reason, path.c_str());
  };

  for (;;) {
    if (g_stop.load(std::memory_order_relaxed)) {
      std::printf("\n  interrupted, closing the recording cleanly\n");
      take_snapshot("interrupted");
      break;
    }
    if (opt.duration_ms > 0 &&
        (ls::now_ns() - t0) / 1000000u >= static_cast<std::uint64_t>(opt.duration_ms)) {
      break;
    }

    ls::sample<RobotRange> s;
    const auto res = range_sub.take(s);
    if (res == ls::read_result::retry) continue;
    if (res != ls::read_result::ok) { ls::sleep_us(2000); continue; }

    RobotRange r{};
    if (!s.copy_out(r)) { ++stale; continue; }  // publisher lapped us mid-read

    // Journal the input BEFORE deciding, so the stream reads: reading, then the
    // clock read that decision made. Replay walks it in the same order.
    if (recording) {
      jr.append(ls::record_kind::message, kRangeCallback, s.sequence(),
                s.stamp_ns(), &r, sizeof(r));
    }

    // Hoisted out of the decide() call so the transcript can timestamp the row.
    // Still exactly one clock read, still in the same position in the journal
    // stream, which is what replay walks -- see run_replay().
    const std::uint64_t now = clk.now_ns();
    const RobotDrive d = decide(r, now, seq++);
    out.mix_value(d.left);
    out.mix_value(d.right);
    ++decisions;

    if (!have_log_t0) { log_t0 = now; have_log_t0 = true; }
    const double elapsed_s = static_cast<double>(now - log_t0) / 1e9;
    const char* note = stats.add(elapsed_s, r);
    write_table_row(stdout, decisions - 1, elapsed_s, r, d, note);
    log.row(decisions - 1, elapsed_s, r, d, note);

    // The trigger. A sensor that has stopped answering for this many reads in
    // a row is the failure this robot actually has, and catching it here is the
    // difference between a snapshot of the fault and a snapshot of whatever the
    // robot was doing when the clock ran out.
    if (opt.blind_trigger > 0 &&
        stats.blind_streak == static_cast<std::uint64_t>(opt.blind_trigger)) {
      take_snapshot("sensor-blind");
    }

    // ---- camera, AFTER the clock read above ----
    //
    // The ordering is load bearing, not stylistic. See kFrameCallback: a record
    // appended between the range reading and the decision's clock read is eaten
    // by the clock's forward scan during replay. Appending here, after that
    // read has already happened, is what keeps frames readable on replay.
    // DRAINED, not sampled once. The bridge captures faster than this loop
    // decides -- that is the point of putting the camera on its own thread --
    // so there are several frames waiting per decision. Taking one would throw
    // the rest away and put the choppiness straight back.
    if (have_camera) {
      ls::sample<RobotFrame> fs;
      while (camera_sub.take(fs) == ls::read_result::ok) {
        if (!fs.copy_out(*frame_buf)) continue;  // publisher lapped us mid-read
        if (frame_buf->bytes == 0 || frame_buf->bytes > kMaxFrameBytes) continue;
        if (recording &&
            !jr.append(ls::record_kind::message, kFrameCallback, fs.sequence(),
                       fs.stamp_ns(), frame_buf->data, frame_buf->bytes)) {
          ++journal_full;  // out of room; keep driving, say so at the end
        }
        if (opt.write_video) {
          video.add(frame_buf->data, frame_buf->bytes, frame_buf->stamp_ns);
        }
      }
    }

    auto ln = drive_pub.loan_message();
    *ln.get() = d;
    drive_pub.publish(ln);
  }

  // Always leave a snapshot behind in ring mode, even on a clean run. The
  // rolling journal itself is a window with rotation bookkeeping in it; the
  // snapshot is the thing you replay.
  take_snapshot("end-of-run");

  const std::string video_note = video.close();
  log.finish(stats, out.value(), opt.journal, video_note, snapshot_note);
  stats.write(stdout, out.value(), opt.journal, video_note, snapshot_note);
  if (stale > 0) {
    std::printf("  (%llu samples dropped because the bridge overtook us)\n",
                static_cast<unsigned long long>(stale));
  }
  if (journal_full > 0) {
    std::printf("  WARNING: the journal filled up and %llu frame(s) were not\n"
                "  recorded. Re-run with a bigger --journal-mb than %d.\n",
                static_cast<unsigned long long>(journal_full), opt.journal_mb);
  }
  if (log.is_open()) {
    std::printf("\n  Saved to %s\n", log.path().c_str());
  }
  return 0;
}
