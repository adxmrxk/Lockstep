// elegoo_bridge : puts a physical Elegoo Smart Robot Car V4 on the bus.
//
// This is the only process that talks to hardware. Everything upstream of it is
// an ordinary Lockstep node that neither knows nor cares that a robot exists,
// which is the point: the control code you write is the same whether it is
// driving a real car, a simulated one, or a recorded journal being replayed.
//
//   [Elegoo car]                        [this computer]
//     ESP32 :100  <--- WiFi/TCP --->  elegoo_bridge  --robot/range-->  your node
//     UNO (motors)                                   <--robot/drive--
//
// MODES
//
//   (default)  connect to the robot and bridge it
//   --sim      no robot; synthesise sensor data and absorb drive commands. The
//              whole graph runs, so you can develop and test control code with
//              the car in its box.
//   --probe    send each command once, print the raw reply, exit. This is how
//              you verify robot/elegoo/protocol.hpp against YOUR firmware --
//              see the warning at the top of that file.
//
// SAFETY
//
// Two independent stops, because a runaway robot is worse than a stopped one:
//
//   1. this process sends an explicit stop on every exit path, including a
//      caught signal;
//   2. the firmware's own failsafe halts the car if it hears nothing for about
//      half a second, which covers the case where this process is killed
//      outright and never gets to run (1).
//
// The heartbeat exists to keep (2) from firing during normal operation, and its
// interval is set well inside that window in protocol.hpp.
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "camera.hpp"
#include "lockstep/core/process.hpp"
#include "lockstep/net/tcp.hpp"
#include "lockstep/pubsub.hpp"
#include "lockstep/shm/liveness.hpp"
#include "protocol.hpp"
#include "robot_msgs.hpp"

namespace {

std::atomic<bool> g_stop{false};
extern "C" void on_signal(int) { g_stop.store(true, std::memory_order_relaxed); }

struct options {
  std::string host = "192.168.4.1";  // the ESP32's address in its own AP mode
  std::uint16_t port = 100;
  std::string bus = "lockstep-robot";
  bool attach = false;   // join an existing bus instead of creating one
  bool sim = false;
  bool probe = false;
  // 4 Hz, not 20. A measured ultrasonic round trip on this car is ~250 ms
  // (query out, {ls_52} back), so a 50 ms loop cannot work: the reply for one
  // cycle would not arrive until four cycles later. 20 Hz was wishful.
  int rate_hz = 4;
  long duration_ms = 0;  // 0 = until interrupted
  int connect_timeout_ms = 3000;
  // Measured replies land at 238-255 ms, so the old 250 ms budget sat exactly
  // on the threshold and dropped readings at random.
  int reply_timeout_ms = 1000;
  // Camera capture is off by default. It is a second HTTP connection per cycle
  // to the same ESP32 that is already the suspect in the mid-drive link death,
  // so turning it on has to be a decision, not a surprise.
  bool camera = false;
  std::uint16_t camera_port = 80;
  std::string camera_path = "/capture";
  // Deliberately NOT tied to rate_hz. The control loop is 4 Hz because a
  // command round trip costs ~250 ms, but a camera fetch is a different socket
  // to a different server and has no such floor. Pinning the two together gave
  // video at 4 fps, in which the car visibly jumps a foot between frames.
  int camera_fps = 12;
};

void usage() {
  std::printf(
      "elegoo_bridge -- put an Elegoo Smart Robot Car V4 on the Lockstep bus\n\n"
      "  --host <ip>        robot address (default 192.168.4.1)\n"
      "  --port <n>         robot TCP port (default 100)\n"
      "  --bus <name>       segment name (default lockstep-robot)\n"
      "  --attach           join an existing bus instead of creating it\n"
      "  --rate <hz>        sensor poll rate (default 4; the car answers in\n"
      "                     ~250ms, so much above 4 Hz is not achievable)\n"
      "  --duration <ms>    run time; 0 or omitted means until Ctrl-C\n"
      "  --camera           also publish camera frames on robot/camera, so a\n"
      "                     recording node can save video of the drive\n"
      "  --camera-port <n>  camera HTTP port (default 80)\n"
      "  --camera-path <p>  camera HTTP path (default /capture)\n"
      "  --camera-fps <n>   capture rate (default 12). Independent of --rate:\n"
      "                     the camera runs on its own thread so that smooth\n"
      "                     video does not require a faster control loop\n"
      "  --sim              no robot: synthesise sensors, absorb drive commands\n"
      "  --probe            send each command once, print raw replies, exit\n");
}

bool parse_args(int argc, char** argv, options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char* what) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s needs a value\n", what);
        return nullptr;
      }
      return argv[++i];
    };
    if (a == "--sim") o.sim = true;
    else if (a == "--probe") o.probe = true;
    else if (a == "--attach") o.attach = true;
    else if (a == "--camera") o.camera = true;
    else if (a == "--camera-port") { const char* v = next("--camera-port");
                                     if (!v) return false;
                                     o.camera_port = static_cast<std::uint16_t>(std::atoi(v)); }
    else if (a == "--camera-path") { const char* v = next("--camera-path");
                                     if (!v) return false; o.camera_path = v; }
    else if (a == "--camera-fps") { const char* v = next("--camera-fps");
                                    if (!v) return false; o.camera_fps = std::atoi(v);
                                    if (o.camera_fps < 1) o.camera_fps = 1; }
    else if (a == "--help" || a == "-h") { usage(); return false; }
    else if (a == "--host") { const char* v = next("--host"); if (!v) return false; o.host = v; }
    else if (a == "--bus") { const char* v = next("--bus"); if (!v) return false; o.bus = v; }
    else if (a == "--port") { const char* v = next("--port"); if (!v) return false;
                              o.port = static_cast<std::uint16_t>(std::atoi(v)); }
    else if (a == "--rate") { const char* v = next("--rate"); if (!v) return false;
                              o.rate_hz = std::atoi(v); }
    else if (a == "--duration") { const char* v = next("--duration"); if (!v) return false;
                                  o.duration_ms = std::atol(v); }
    else { std::fprintf(stderr, "unknown option: %s\n", a.c_str()); usage(); return false; }
  }
  if (o.rate_hz < 1) o.rate_hz = 1;
  return true;
}

// ---------------------------------------------------------------------------
// The link to the robot. One object so that "real robot" and "no robot" are the
// same shape to the rest of the file, and the main loop has no idea which it is
// driving.
//
// Named robot_link rather than link because POSIX already has a link(2) that
// arrives via <unistd.h>, and the collision is silent until it is not.
// ---------------------------------------------------------------------------
class robot_link {
 public:
  robot_link(const options& o) : opt_(o) {}

  bool open() {
    if (opt_.sim) {
      connected_ = true;
      std::printf("  [link] SIMULATED -- no robot, synthetic sensors\n");
      return true;
    }
    std::string err;
    stream_ = ls::net::tcp_stream::connect(opt_.host, opt_.port,
                                           opt_.connect_timeout_ms, &err);
    connected_ = stream_.valid();
    if (!connected_) {
      std::fprintf(stderr, "  [link] cannot reach %s:%u -- %s\n", opt_.host.c_str(),
                   static_cast<unsigned>(opt_.port), err.c_str());
      return false;
    }
    std::printf("  [link] connected to %s:%u\n", opt_.host.c_str(),
                static_cast<unsigned>(opt_.port));
    return true;
  }

  bool connected() const { return connected_; }
  std::uint64_t sent() const { return sent_; }
  std::uint64_t ok() const { return ok_; }
  std::uint64_t errors() const { return errors_; }

  // Send a command and read its reply. In simulation the reply is manufactured
  // so the call has the same shape and the same failure modes.
  ls::elegoo::reply exchange(const std::string& cmd, bool expect_reply) {
    ++sent_;
    if (opt_.sim) return simulate(cmd);

    // No trailing newline: the firmware frames on braces and a newline is
    // simply junk between frames.
    if (!stream_.valid() || !stream_.send_all(cmd)) {
      ++errors_;
      connected_ = false;
      return {};
    }
    if (!expect_reply) {
      ++ok_;
      service_heartbeat();
      return {true, false, 0, ""};
    }

    // The car's 1 Hz {Heartbeat} shares this socket with command replies, so
    // read until a frame arrives that is not one. Handing a heartbeat to
    // parse_reply would be worse than losing the reply: it parses as
    // affirmative-with-no-value, the caller books it as the answer, and every
    // later reply is then matched to the wrong request.
    const std::uint64_t t0 = ls::now_ns();
    for (;;) {
      const long spent = static_cast<long>((ls::now_ns() - t0) / 1000000u);
      const int left = opt_.reply_timeout_ms - static_cast<int>(spent);
      if (left <= 0) break;
      std::string frame;
      if (!stream_.recv_frame(frame, left)) break;
      if (ls::elegoo::is_heartbeat(frame)) { answer_heartbeat(); continue; }
      ++ok_;
      return ls::elegoo::parse_reply(frame);
    }
    // A missed reply is not fatal on its own -- the firmware drops replies
    // under load -- but a run of them means the link is gone.
    ++errors_;
    return {};
  }

  // Answer whatever keepalive is already waiting, without blocking.
  //
  // This cannot live only in the reply path. The ESP32 drops the session after
  // three unanswered heartbeats, so an iteration that happened to send nothing
  // still has to service the socket or the link dies on an idle robot.
  void service_heartbeat() {
    if (opt_.sim) return;
    std::string frame;
    while (stream_.valid() && stream_.recv_frame(frame, 1)) {
      if (ls::elegoo::is_heartbeat(frame)) answer_heartbeat();
    }
  }

  void answer_heartbeat() {
    if (!stream_.send_all(ls::elegoo::heartbeat_frame)) {
      connected_ = false;
      ++errors_;
    }
  }

 private:
  // A believable world: an obstacle the robot approaches and then backs away
  // from, plus the occasional bad reading, because the real sensor produces
  // those constantly and control code that has never seen one is not tested.
  ls::elegoo::reply simulate(const std::string& cmd) {
    ++ok_;
    if (cmd.find("\"N\":21") == std::string::npos) return {true, false, 0, "{ok}"};

    ++sim_tick_;
    long cm = 0;
    const long phase = sim_tick_ % 200;
    cm = phase < 100 ? (120 - phase) : (20 + (phase - 100));
    if (sim_tick_ % 37 == 0) cm = 0;  // the misread the HC-SR04 loves
    return {true, true, cm, "{" + std::to_string(cm) + "}"};
  }

  options opt_;
  ls::net::tcp_stream stream_;
  bool connected_ = false;
  std::uint64_t sent_ = 0, ok_ = 0, errors_ = 0;
  long sim_tick_ = 0;
};

// ---------------------------------------------------------------------------
// --probe: show what this firmware actually answers.
// ---------------------------------------------------------------------------
int run_probe(robot_link& l) {
  const ls::elegoo::profile& p = ls::elegoo::default_profile;
  struct probe_item { const char* what; std::string cmd; bool reply; };
  const probe_item items[] = {
      {"heartbeat (keepalive)", ls::elegoo::encode_heartbeat(p), false},
      {"ultrasonic", ls::elegoo::encode_ultrasonic(p), true},
      {"line track", ls::elegoo::encode_line_track(p), true},
      {"stop", ls::elegoo::encode_stop(p), true},
      {"move fwd (slow)", ls::elegoo::encode_move(ls::elegoo::direction::forward, 100, p), true},
      {"stop", ls::elegoo::encode_stop(p), true},
      {"left motor", ls::elegoo::encode_left(120, p), true},
      {"stop", ls::elegoo::encode_stop(p), true},
  };

  std::printf("\n  %-22s %-40s %s\n", "COMMAND", "SENT", "RAW REPLY");
  std::printf("  %-22s %-40s %s\n", "-------", "----", "---------");
  for (const auto& it : items) {
    const ls::elegoo::reply r = l.exchange(it.cmd, it.reply);
    // Silence on the keepalive is correct, not a fault: it is the thing we
    // send BACK, so reporting it as a timeout reads like a broken car.
    const char* shown =
        !r.raw.empty() ? r.raw.c_str()
                       : (it.reply ? "(no reply / timeout)"
                                   : "(keepalive -- none expected)");
    std::printf("  %-22s %-40s %s\n", it.what, it.cmd.c_str(), shown);
    ls::sleep_us(300000);
  }

  std::printf(
      "\n  If a reply is empty or nonsense, the command number in\n"
      "  robot/elegoo/protocol.hpp does not match your firmware. Edit the\n"
      "  `profile` struct there -- nothing else needs to change.\n"
      "\n  If EVERY row is empty but the link connected, check the slide switch\n"
      "  on the shield: it has to be in the camera/ESP32 position, not the\n"
      "  upload position, or the ESP32 is cut off from the Arduino and the\n"
      "  WiFi side works perfectly while nothing reaches the motors.\n");
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  options opt;
  if (!parse_args(argc, argv, opt)) return 1;

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  std::printf("Elegoo bridge\n=============\n");

  robot_link l(opt);
  if (!l.open() && !opt.sim) {
    std::fprintf(stderr,
                 "\n  The robot is not reachable. Check that:\n"
                 "    - the car is powered on\n"
                 "    - you are joined to its WiFi access point\n"
                 "    - --host matches its address (192.168.4.1 in AP mode)\n"
                 "\n  You can develop without it: re-run with --sim\n");
    return 1;
  }

  if (opt.probe) return run_probe(l);

  // ------------------------------------------------------------------ bus --
  ls::bus b;
  try {
    if (opt.attach) {
      b = ls::bus::open(opt.bus);
    } else {
      ls::segment::unlink(opt.bus);
      // The 64 KB pool holds camera frames, and it needs more blocks than the
      // camera ring is deep -- a ring of N claims N blocks, so a pool of
      // exactly N leaves nothing and create() fails. 40 covers a 16-deep ring
      // with room to spare.
      b = ls::bus::create(opt.bus, {{64, 512}, {4096, 64}, {65536, 40}});
    }
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "  [bus] %s\n", ex.what());
    return 1;
  }
  std::printf("  [bus] %s, %llu bytes\n", opt.bus.c_str(),
              static_cast<unsigned long long>(b.seg().size()));

  // Wrapped, because an uncaught exception here exits the process after the
  // "[bus] ..." line with NO message at all -- which is exactly how a ring
  // that would not fit in its pool presented itself: a bridge that appeared to
  // start and then simply was not there.
  ls::publisher<RobotRange> range_pub;
  ls::publisher<RobotStatus> status_pub;
  try {
    range_pub = ls::publisher<RobotRange>::create(b, robot_topics::range, 64);
    status_pub = ls::publisher<RobotStatus>::create(b, robot_topics::status, 16);
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "  [bus] cannot create the sensor publishers: %s\n",
                 ex.what());
    return 1;
  }

  // ---- the camera, on its own thread ----
  //
  // A fetch blocks for as long as the ESP32 takes to encode and ship a JPEG,
  // which is tens to hundreds of milliseconds and wholly unrelated to the
  // control loop's business. Running it inline put the camera's latency into
  // the steering path AND held the capture rate down to the decision rate, so
  // video came out at 4 fps with the car lurching between frames.
  //
  // On its own thread it captures as fast as the link allows, the control loop
  // is untouched, and the two rates are free to differ -- which is the whole
  // reason the recorder drains the topic rather than sampling it once.
  //
  // Thread safety: this thread is the ONLY toucher of camera_pub. The main loop
  // never looks at it. One publisher, one thread, no sharing.
  ls::publisher<RobotFrame> camera_pub;
  ls::camera::options cam_opt;
  std::atomic<std::uint64_t> frames_ok{0}, frames_failed{0};
  std::atomic<bool> camera_stop{false};
  std::thread camera_thread;

  if (opt.camera) {
    try {
      camera_pub = ls::publisher<RobotFrame>::create(b, robot_topics::camera, 16);
    } catch (const std::exception& ex) {
      std::fprintf(stderr,
                   "  [cam] cannot create the camera topic: %s\n"
                   "        A frame needs a 64 KB slot. If you are running with"
                   " --attach, the\n"
                   "        bus was created by an older build whose pool is too"
                   " small; restart\n"
                   "        the owning bridge.\n", ex.what());
      return 1;
    }
    cam_opt.host = opt.host;
    cam_opt.port = opt.camera_port;
    cam_opt.path = opt.camera_path;
    cam_opt.max_bytes = kMaxFrameBytes;
    std::printf("  [cam] http://%s:%u%s at up to %d fps\n", cam_opt.host.c_str(),
                static_cast<unsigned>(cam_opt.port), cam_opt.path.c_str(),
                opt.camera_fps);
  }

  // The topic exists in --sim so the graph has the same shape either way, but
  // there is nothing to fetch from, and a capture loop against an absent host
  // would just burn a 1.5 second connect timeout over and over.
  if (opt.camera && opt.sim) {
    std::printf("  [cam] SIMULATED -- no camera to fetch from, no frames\n");
  } else if (opt.camera) {
    camera_thread = std::thread([&] {
      const std::uint64_t period_ns =
          1000000000ull / static_cast<unsigned>(opt.camera_fps);
      std::string last_error;
      std::uint32_t cseq = 0;
      while (!camera_stop.load(std::memory_order_relaxed) &&
             !g_stop.load(std::memory_order_relaxed)) {
        const std::uint64_t started = ls::now_ns();
        const ls::camera::result cr = ls::camera::fetch_jpeg(cam_opt);

        if (cr.ok) {
          frames_ok.fetch_add(1, std::memory_order_relaxed);
          auto ln = camera_pub.loan_message();
          ln->stamp_ns = ls::now_ns();
          ln->seq = cseq++;
          ln->bytes = static_cast<std::uint32_t>(cr.body.size());
          std::memcpy(ln->data, cr.body.data(), cr.body.size());
          // The tail of the slot is left as the pool found it. Nothing reads
          // past `bytes`, and zeroing 60 KB a frame would cost more than the
          // fetch did.
          camera_pub.publish(ln);
          last_error.clear();
        } else {
          frames_failed.fetch_add(1, std::memory_order_relaxed);
          // Only announce a NEW failure. A camera that is off would otherwise
          // print a dozen identical lines a second and bury the drive.
          if (cr.error != last_error) {
            std::printf("  [cam] %s\n", cr.error.c_str());
            last_error = cr.error;
          }
        }

        // Pace to the requested rate. A fetch slower than the period means the
        // link is the limit, so there is nothing to wait for -- go again.
        const std::uint64_t spent = ls::now_ns() - started;
        if (spent < period_ns) ls::sleep_us((period_ns - spent) / 1000u);
      }
    });
  }

  // The drive topic is published by whatever control node the user writes, so
  // the bridge has to tolerate it not existing yet -- and has to keep working
  // if that node is killed and restarted, which is the whole point of the
  // reaper. Subscribing lazily handles both.
  bool have_drive = false;
  ls::subscriber<RobotDrive> drive_sub;

  std::printf("  [bus] publishing %s, %s; subscribing %s\n", robot_topics::range,
              robot_topics::status, robot_topics::drive);
  std::printf("  [run] %d Hz, %s\n\n", opt.rate_hz,
              opt.duration_ms ? (std::to_string(opt.duration_ms) + " ms").c_str()
                              : "until Ctrl-C");

  const unsigned period_us = static_cast<unsigned>(1000000 / opt.rate_hz);
  const std::uint64_t t0 = ls::now_ns();

  // Somebody has to run the reaper, and in a real deployment it had better be a
  // process that outlives the nodes it is reclaiming for. The bridge owns the
  // bus and runs for the whole session, so it is the right owner.
  //
  // Without this, a publisher that dies keeps its topic forever and the
  // replacement is refused with "topic already has a publisher" -- which is
  // precisely the failure the reaper was written to prevent, left unarmed
  // because nothing called it. A five-hour soak with node churn found this
  // within two minutes.
  ls::reaper reaper(b);
  std::uint64_t last_reap = t0;
  constexpr std::uint64_t kReapIntervalMs = 1000;
  std::uint64_t reclaimed_total = 0;
  std::uint32_t seq = 0;
  std::uint64_t commands_forwarded = 0;

  while (!g_stop.load(std::memory_order_relaxed)) {
    if (opt.duration_ms > 0 &&
        (ls::now_ns() - t0) / 1000000u >= static_cast<std::uint64_t>(opt.duration_ms)) {
      break;
    }

    // ---- sensors out ----
    const ls::elegoo::reply rr = l.exchange(ls::elegoo::encode_ultrasonic(), true);
    {
      auto ln = range_pub.loan_message();
      ln->stamp_ns = ls::now_ns();
      ln->seq = seq;
      const bool good = rr.has_value && ls::elegoo::range_is_plausible(rr.value);
      ln->valid = good ? 1u : 0u;
      ln->distance_cm = good ? static_cast<float>(rr.value) : 0.0f;
      range_pub.publish(ln);
    }

    // ---- commands in ----
    if (!have_drive) {
      try {
        drive_sub = ls::subscriber<RobotDrive>::attach(b, robot_topics::drive);
        have_drive = true;
        std::printf("  [bus] a control node appeared on %s\n", robot_topics::drive);
      } catch (const std::exception&) {
        // Nobody is driving yet. Normal at startup.
      }
    }
    if (have_drive) {
      // Drain to the NEWEST command. A control loop wants the freshest
      // intention, not a backlog of stale ones -- replaying an old turn because
      // it queued up is exactly how a robot drives into something.
      RobotDrive latest{};
      bool got = false;
      for (;;) {
        ls::sample<RobotDrive> s;
        const auto res = drive_sub.take(s);
        if (res == ls::read_result::ok) {
          RobotDrive copy{};
          if (s.copy_out(copy)) { latest = copy; got = true; }
        } else if (res == ls::read_result::retry ||
                   res == ls::read_result::overrun) {
          // An overrun is NOT a reason to stop draining. read() has already
          // moved the cursor forward to the oldest resident message, so the
          // right move is to go round again and consume from there. Breaking
          // here instead is a trap: by the next cycle write_pos has advanced,
          // the cursor is once again behind the trailing edge, and it overruns
          // again -- forever, consuming nothing. A 5-hour soak caught this as
          // 339,109 overruns against 353,428 messages, a 96% loss rate, at
          // exactly one overrun per loop iteration.
          continue;
        } else {
          break;  // empty or abandoned
        }
      }
      if (got) {
        // Wait for each acknowledgement rather than firing and forgetting.
        //
        // These DO answer -- {ls_ok}, about 250ms later. Sending them without
        // collecting the reply leaves it sitting in the socket, and the next
        // cycle's range read picks up that stale {ls_ok} instead of its own
        // {ls_44}: no number in it, so the reading is marked invalid, and from
        // then on every reply is matched to the request before it. A correct
        // 1.3 Hz beats a 4 Hz that is reading one cycle behind reality.
        l.exchange(ls::elegoo::encode_left(latest.left), true);
        l.exchange(ls::elegoo::encode_right(latest.right), true);
        ++commands_forwarded;
      }
    }

    // ---- keep the session alive ----
    // The car drives this, not us: it sends {Heartbeat} every second and hangs
    // up after three unanswered. exchange() answers them inline, but an
    // iteration that sent nothing still has to drain the socket.
    const std::uint64_t now = ls::now_ns();
    l.service_heartbeat();

    // ---- reclaim anything that died ----
    if ((now - last_reap) / 1000000u >= kReapIntervalMs) {
      const ls::reap_report rr = reaper.scan();
      if (rr.publishers_reclaimed || rr.slots_healed) {
        reclaimed_total += rr.publishers_reclaimed;
        std::printf("  [reap] reclaimed %u publisher(s), healed %u slot(s)\n",
                    rr.publishers_reclaimed, rr.slots_healed);
      }
      last_reap = now;
    }

    // ---- link health ----
    {
      auto ln = status_pub.loan_message();
      ln->stamp_ns = now;
      ln->seq = seq;
      ln->connected = l.connected() ? 1u : 0u;
      ln->commands_sent = l.sent();
      ln->replies_ok = l.ok();
      ln->errors = l.errors();
      status_pub.publish(ln);
    }

    ++seq;
    ls::sleep_us(period_us);
  }

  // Stop the camera before anything else. It is holding a socket to the same
  // ESP32 the stop command has to reach, and the stop is the one message in
  // this program that must not be delayed.
  camera_stop.store(true, std::memory_order_relaxed);
  if (camera_thread.joinable()) camera_thread.join();

  // Safety stop. Belt; the firmware failsafe is braces.
  l.exchange(ls::elegoo::encode_stop(), false);

  std::printf(
      "\n  [done] %u cycles, %llu drive commands forwarded, "
      "%llu publisher(s) reclaimed\n"
      "         link: %llu sent, %llu ok, %llu errors, %s\n",
      seq, static_cast<unsigned long long>(commands_forwarded),
      static_cast<unsigned long long>(reclaimed_total),
      static_cast<unsigned long long>(l.sent()),
      static_cast<unsigned long long>(l.ok()),
      static_cast<unsigned long long>(l.errors()),
      l.connected() ? "connected" : "DISCONNECTED");
  if (opt.camera) {
    const std::uint64_t got = frames_ok.load(std::memory_order_relaxed);
    const double secs = static_cast<double>(ls::now_ns() - t0) / 1e9;
    std::printf("         camera: %llu frames, %llu failed, %.1f fps achieved\n",
                static_cast<unsigned long long>(got),
                static_cast<unsigned long long>(
                    frames_failed.load(std::memory_order_relaxed)),
                secs > 0.0 ? static_cast<double>(got) / secs : 0.0);
  }
  return 0;
}
