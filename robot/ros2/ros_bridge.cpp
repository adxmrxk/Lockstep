// ros_bridge : put an existing ROS 2 system on the Lockstep bus.
//
// This is the node that changes what Lockstep is for. Without it the project is
// a replacement for the ROS 2 transport, which invites the fair question of why
// anyone would migrate a working stack onto it. With it, Lockstep is something
// you ADD to a ROS 2 system:
//
//     your ROS 2 nodes  --/imu/data-->  ros_bridge  --ros/imu-->  Lockstep
//                       <--/cmd_vel--               <--ros/cmd_vel--
//
// and what you get in return is the thing ROS 2 cannot do for itself -- a
// recording that replays bit for bit, because the journal captures dispatch
// order and clock reads and not just the message payloads.
//
//   ros_bridge --in imu:/imu/data --out twist:/cmd_vel
//
// DIRECTIONS
//   --in  <type>:<ros_topic>   subscribe in ROS, publish into Lockstep
//   --out <type>:<ros_topic>   subscribe in Lockstep, publish into ROS
//
// Types: imu, twist, range. Each is a hand-written conversion -- see the note
// at the top of ros_msgs.hpp for why a generic bridge is not possible.
//
// A note on QoS. Sensor topics are bridged with SensorDataQoS (best effort,
// keep last), which is what sensor publishers actually use; a RELIABLE
// subscriber silently fails to match a BEST_EFFORT publisher, and that mismatch
// is the single most common reason a ROS 2 subscription receives nothing.
// Command topics use the reliable default, because a dropped velocity command
// is a robot that keeps driving.
#include <atomic>
#include <csignal>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "convert.hpp"
#include "lockstep/core/process.hpp"
#include "lockstep/pubsub.hpp"
#include "lockstep/replay/journal.hpp"
#include "ros_msgs.hpp"

namespace {

std::atomic<bool> g_stop{false};
extern "C" void on_signal(int) { g_stop.store(true, std::memory_order_relaxed); }

struct binding {
  std::string type;       // imu | twist | range
  std::string ros_topic;
  bool inbound = true;    // true: ROS -> Lockstep
};

struct options {
  std::string bus = "lockstep-ros";
  std::string frame_id = "base_link";
  std::string journal;
  bool attach = false;
  long duration_ms = 0;
  std::vector<binding> bindings;
};

void usage() {
  std::printf(
      "ros_bridge -- bridge ROS 2 topics onto the Lockstep bus\n\n"
      "  --in  <type>:<topic>   ROS -> Lockstep   (e.g. imu:/imu/data)\n"
      "  --out <type>:<topic>   Lockstep -> ROS   (e.g. twist:/cmd_vel)\n"
      "  --bus <name>           segment name (default lockstep-ros)\n"
      "  --attach               join an existing bus instead of creating one\n"
      "  --frame <id>           frame_id to stamp outbound ROS messages with\n"
      "  --journal <path>       record everything bridged, for replay\n"
      "  --duration <ms>        run time; 0 means until Ctrl-C\n\n"
      "types: imu, twist, range\n");
}

bool split_binding(const std::string& arg, binding& b, bool inbound) {
  const auto colon = arg.find(':');
  if (colon == std::string::npos || colon == 0 || colon + 1 >= arg.size()) {
    std::fprintf(stderr, "expected <type>:<topic>, got '%s'\n", arg.c_str());
    return false;
  }
  b.type = arg.substr(0, colon);
  b.ros_topic = arg.substr(colon + 1);
  b.inbound = inbound;
  if (b.type != "imu" && b.type != "twist" && b.type != "range") {
    std::fprintf(stderr, "unknown type '%s' (want imu, twist or range)\n",
                 b.type.c_str());
    return false;
  }
  return true;
}

// One bridged topic. Holds the ROS endpoint, the Lockstep endpoint, and the
// counters the summary prints.
template <class RosMsg, class LsMsg>
struct inbound_link {
  typename rclcpp::Subscription<RosMsg>::SharedPtr sub;
  std::unique_ptr<ls::publisher<LsMsg>> pub;
  std::uint64_t count = 0;
};

template <class RosMsg, class LsMsg>
struct outbound_link {
  typename rclcpp::Publisher<RosMsg>::SharedPtr pub;
  std::unique_ptr<ls::subscriber<LsMsg>> sub;
  std::uint64_t count = 0;
};

}  // namespace

int main(int argc, char** argv) {
  options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
    if (a == "--help" || a == "-h") { usage(); return 0; }
    else if (a == "--attach") opt.attach = true;
    else if (a == "--bus") { const char* v = next(); if (!v) return 1; opt.bus = v; }
    else if (a == "--frame") { const char* v = next(); if (!v) return 1; opt.frame_id = v; }
    else if (a == "--journal") { const char* v = next(); if (!v) return 1; opt.journal = v; }
    else if (a == "--duration") { const char* v = next(); if (!v) return 1;
                                  opt.duration_ms = std::atol(v); }
    else if (a == "--in" || a == "--out") {
      const char* v = next();
      if (!v) return 1;
      binding b;
      if (!split_binding(v, b, a == "--in")) return 1;
      opt.bindings.push_back(b);
    } else if (a.rfind("--ros-args", 0) == 0) {
      break;  // everything from here is rclcpp's
    } else {
      std::fprintf(stderr, "unknown option: %s\n", a.c_str());
      usage();
      return 1;
    }
  }

  if (opt.bindings.empty()) {
    std::fprintf(stderr, "nothing to bridge; pass at least one --in or --out\n\n");
    usage();
    return 1;
  }

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lockstep_bridge");

  std::printf("ROS 2 bridge\n============\n");

  // ------------------------------------------------------------------ bus --
  ls::bus b;
  try {
    if (opt.attach) {
      b = ls::bus::open(opt.bus);
    } else {
      ls::segment::unlink(opt.bus);
      // The size classes matter here. RosImu is 96 bytes, so without a class
      // between 64 B and 4 KB every message slot would take a 4 KB block and a
      // 128-slot ring would exhaust the pool before it finished starting. The
      // 256 B class exists precisely for the bridged message types.
      b = ls::bus::create(opt.bus, {{64, 256},
                                    {256, 1024},
                                    {4096, 64},
                                    {65536, 16}});
    }
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "  [bus] %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  std::printf("  [bus] %s, %llu bytes\n", opt.bus.c_str(),
              static_cast<unsigned long long>(b.seg().size()));

  ls::journal jr;
  const bool recording = !opt.journal.empty();
  if (recording) {
    // Sized for a long run: at a few hundred bytes per record this holds hours
    // of a busy graph. The journal does not rotate, so it is sized rather than
    // rolled, and that limit is real -- see the note in replay/journal.hpp.
    jr = ls::journal::create(opt.journal, 256u << 20);
    std::printf("  [rec] journalling to %s\n", opt.journal.c_str());
  }

  // Sensor data is published best-effort by convention; subscribing RELIABLE to
  // a BEST_EFFORT publisher matches nothing and looks exactly like a dead topic.
  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto command_qos = rclcpp::QoS(10);

  inbound_link<sensor_msgs::msg::Imu, RosImu> in_imu;
  inbound_link<sensor_msgs::msg::Range, RosRange> in_range;
  outbound_link<geometry_msgs::msg::Twist, RosTwist> out_twist;
  std::uint32_t imu_seq = 0, range_seq = 0;
  std::uint64_t journalled = 0;

  try {
  for (const auto& bind : opt.bindings) {
    if (bind.inbound && bind.type == "imu") {
      in_imu.pub = std::make_unique<ls::publisher<RosImu>>(
          ls::publisher<RosImu>::create(b, ros_topics::imu, 128));
      in_imu.sub = node->create_subscription<sensor_msgs::msg::Imu>(
          bind.ros_topic, sensor_qos,
          [&](const sensor_msgs::msg::Imu::SharedPtr m) {
            const RosImu conv = ls::ros::from_ros(*m, imu_seq++);
            auto ln = in_imu.pub->loan_message();
            *ln.get() = conv;
            in_imu.pub->publish(ln);
            ++in_imu.count;
            if (recording &&
                jr.append(ls::record_kind::message, 0, conv.seq, conv.stamp_ns,
                          &conv, sizeof(conv))) {
              ++journalled;
            }
          });
      std::printf("  [in ] ROS %s  -->  lockstep %s\n", bind.ros_topic.c_str(),
                  ros_topics::imu);

    } else if (bind.inbound && bind.type == "range") {
      in_range.pub = std::make_unique<ls::publisher<RosRange>>(
          ls::publisher<RosRange>::create(b, ros_topics::range, 128));
      in_range.sub = node->create_subscription<sensor_msgs::msg::Range>(
          bind.ros_topic, sensor_qos,
          [&](const sensor_msgs::msg::Range::SharedPtr m) {
            const RosRange conv = ls::ros::from_ros(*m, range_seq++);
            auto ln = in_range.pub->loan_message();
            *ln.get() = conv;
            in_range.pub->publish(ln);
            ++in_range.count;
            if (recording &&
                jr.append(ls::record_kind::message, 1, conv.seq, conv.stamp_ns,
                          &conv, sizeof(conv))) {
              ++journalled;
            }
          });
      std::printf("  [in ] ROS %s  -->  lockstep %s\n", bind.ros_topic.c_str(),
                  ros_topics::range);

    } else if (!bind.inbound && bind.type == "twist") {
      out_twist.pub = node->create_publisher<geometry_msgs::msg::Twist>(
          bind.ros_topic, command_qos);
      try {
        out_twist.sub = std::make_unique<ls::subscriber<RosTwist>>(
            ls::subscriber<RosTwist>::attach(b, ros_topics::twist));
      } catch (const std::exception&) {
        // Nobody is publishing commands into Lockstep yet. Attach lazily in the
        // spin loop instead of refusing to start.
      }
      std::printf("  [out] lockstep %s  -->  ROS %s\n", ros_topics::twist,
                  bind.ros_topic.c_str());

    } else {
      std::fprintf(stderr, "  unsupported direction for type '%s'\n",
                   bind.type.c_str());
      rclcpp::shutdown();
      return 1;
    }
  }
  } catch (const std::exception& ex) {
    // Almost always a pool too small for the message size or the ring depth.
    // An uncaught throw here used to reach terminate and core-dump, which told
    // the operator nothing about what was actually wrong.
    std::fprintf(stderr, "  [setup] failed: %s\n", ex.what());
    rclcpp::shutdown();
    return 1;
  }

  std::printf("  [run] %s\n\n",
              opt.duration_ms ? (std::to_string(opt.duration_ms) + " ms").c_str()
                              : "until Ctrl-C");

  // ----------------------------------------------------------- spin loop --
  const std::uint64_t t0 = ls::now_ns();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while (rclcpp::ok() && !g_stop.load(std::memory_order_relaxed)) {
    if (opt.duration_ms > 0 &&
        (ls::now_ns() - t0) / 1000000u >= static_cast<std::uint64_t>(opt.duration_ms)) {
      break;
    }

    // Pump ROS callbacks, which feed the inbound direction.
    exec.spin_some(std::chrono::milliseconds(5));

    // Drain Lockstep, which feeds the outbound direction.
    if (out_twist.pub) {
      if (!out_twist.sub) {
        try {
          out_twist.sub = std::make_unique<ls::subscriber<RosTwist>>(
              ls::subscriber<RosTwist>::attach(b, ros_topics::twist));
          std::printf("  [out] a publisher appeared on %s\n", ros_topics::twist);
        } catch (const std::exception&) {
        }
      }
      if (out_twist.sub) {
        for (;;) {
          ls::sample<RosTwist> s;
          const auto rr = out_twist.sub->take(s);
          // Overrun keeps draining, for the same reason as everywhere else:
          // read() has moved the cursor to the oldest resident message and
          // stopping here would pin it behind the trailing edge forever.
          if (rr == ls::read_result::retry ||
              rr == ls::read_result::overrun) continue;
          if (rr != ls::read_result::ok) break;
          RosTwist copy{};
          if (!s.copy_out(copy)) break;  // lapped mid-read; skip it
          out_twist.pub->publish(ls::ros::to_ros(copy));
          ++out_twist.count;
        }
      }
    }
  }

  std::printf("\n  [done] imu %llu, range %llu bridged in; twist %llu bridged out\n",
              static_cast<unsigned long long>(in_imu.count),
              static_cast<unsigned long long>(in_range.count),
              static_cast<unsigned long long>(out_twist.count));
  if (recording) {
    std::printf("  [done] %llu records journalled to %s\n",
                static_cast<unsigned long long>(journalled), opt.journal.c_str());
  }

  rclcpp::shutdown();
  return 0;
}
