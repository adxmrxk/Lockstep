// lockstep top : look inside a running bus.
//
// Until now there was no way to see what a live Lockstep segment was doing. You
// could read the code, run the tests, and watch a demo print its own summary at
// exit -- but if a node was misbehaving right now, there was nothing to look at.
// That is the difference between a library and a tool, and this closes it.
//
//   lockstep_top --bus lockstep-robot
//
// Everything it shows already existed in the segment; nothing here required new
// bookkeeping on the hot path. The registry knows every topic, its layout hash
// and its publisher. Each ring knows its write position and its overrun count.
// The arena knows how many blocks of each class are out. This process maps the
// same segment and reports what it finds.
//
// STRICTLY AN OBSERVER. It never announces a topic, never publishes, never
// takes a slot, and never writes a byte of the segment. A monitoring tool that
// perturbs the thing it monitors is worse than no tool -- and on this bus it
// would be worse than usual, because announcing a topic in order to inspect it
// would claim the exclusive publisher slot and lock out the real publisher.
//
// Message RATE is derived rather than stored: write_pos is sampled between
// refreshes and differenced, which costs the bus nothing.
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "lockstep/core/clock.hpp"
#include "lockstep/core/process.hpp"
#include "lockstep/shm/bus.hpp"
#include "lockstep/shm/ring.hpp"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace {

struct options {
  std::string bus = "lockstep-robot";
  int interval_ms = 500;
  bool once = false;
  bool no_colour = false;
  bool csv = false;
};

void usage() {
  std::printf(
      "lockstep_top -- live view of a running Lockstep bus\n\n"
      "  --bus <name>       segment to attach to (default lockstep-robot)\n"
      "  --interval <ms>    refresh period (default 500)\n"
      "  --once             print one snapshot and exit (for scripts and CI)\n"
      "  --no-colour        plain output, no ANSI colour\n");
}

// Windows consoles need ANSI escapes switched on explicitly; without this the
// screen-clear and colours would print as literal garbage.
void enable_ansi() {
#if defined(_WIN32)
  HANDLE h = ::GetStdHandle(STD_OUTPUT_HANDLE);
  DWORD mode = 0;
  if (h != INVALID_HANDLE_VALUE && ::GetConsoleMode(h, &mode))
    ::SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
#endif
}

struct palette {
  const char* reset = "\033[0m";
  const char* dim = "\033[2m";
  const char* bold = "\033[1m";
  const char* green = "\033[32m";
  const char* red = "\033[31m";
  const char* yellow = "\033[33m";
  const char* cyan = "\033[36m";
};
palette g_c;

void strip_colour() {
  g_c.reset = g_c.dim = g_c.bold = "";
  g_c.green = g_c.red = g_c.yellow = g_c.cyan = "";
}

std::string human_bytes(std::uint64_t n) {
  char buf[32];
  if (n >= (1u << 20)) std::snprintf(buf, sizeof buf, "%.1f MB", n / 1048576.0);
  else if (n >= (1u << 10)) std::snprintf(buf, sizeof buf, "%.1f KB", n / 1024.0);
  else std::snprintf(buf, sizeof buf, "%llu B", static_cast<unsigned long long>(n));
  return buf;
}

// A 10-cell bar. The shape of pool pressure is easier to read at a glance than
// the ratio it is drawn from.
std::string bar(std::uint64_t used, std::uint64_t total, int width = 10) {
  if (total == 0) return std::string(static_cast<std::size_t>(width), '-');
  const int filled = static_cast<int>((used * width + total - 1) / total);
  std::string s;
  for (int i = 0; i < width; ++i) s += (i < filled) ? '#' : '.';
  return s;
}

// Per-topic state carried between refreshes, so a rate can be differenced.
struct sample {
  std::uint64_t write_pos = 0;
  std::uint64_t at_ns = 0;
  bool seen = false;
};

// One pass over the segment. `quiet` collects samples without printing, which
// is how --once primes itself for a rate.
void render(ls::bus& b, std::vector<sample>& prev, const options& opt,
            std::uint64_t started, bool quiet) {
  const std::uint64_t now = ls::now_ns();

  if (!quiet && opt.csv) {
    // Machine-readable. Two record shapes, distinguished by the first column,
    // so a soak log can be appended to for hours and parsed afterwards.
    //   topic,<ts_ns>,<name>,<pid>,<alive>,<messages>,<rate>,<capacity>,<overruns>
    //   arena,<ts_ns>,<pool>,<block_size>,<live>,<block_count>
  } else if (!quiet) {
    if (!opt.once) std::printf("\033[2J\033[H");  // clear, cursor home
    std::printf("%s%slockstep top%s  bus %s%s%s   segment %s   watching %llus\n",
                g_c.bold, g_c.cyan, g_c.reset, g_c.bold, opt.bus.c_str(), g_c.reset,
                human_bytes(b.seg().size()).c_str(),
                static_cast<unsigned long long>((now - started) / 1000000000ull));
    std::printf("\n%s%-22s %8s %8s %10s %9s %7s %9s %18s%s\n",
                g_c.dim, "TOPIC", "PUB PID", "STATE", "MESSAGES", "RATE/s",
                "RING", "OVERRUNS", "LAYOUT HASH", g_c.reset);
  }

  ls::registry& reg = b.topics();
  int shown = 0;
  for (std::uint32_t i = 0; i < reg.capacity(); ++i) {
    ls::topic_entry& e = reg.entry(i);
    if (e.state.load(std::memory_order_acquire) !=
        static_cast<std::uint32_t>(ls::topic_state::ready)) {
      continue;
    }
    ++shown;

    const std::uint32_t pid = e.publisher_pid.load(std::memory_order_acquire);
    const bool alive = pid != 0 && ls::process_alive(pid);

    // A topic can exist with no ring yet: the publisher announced it but has
    // not built the ring. That is a real, brief startup state, not an error.
    const std::uint64_t ro = e.ring_offset.load(std::memory_order_acquire);
    std::uint64_t wp = 0, ov = 0, cap = 0;
    if (ro != 0) {
      ls::ring r(b.seg().at<ls::ring_header>(ro));
      wp = r.write_pos();
      ov = r.overruns();
      cap = r.capacity();
    }

    double rate = -1.0;
    if (prev[i].seen && now > prev[i].at_ns && wp >= prev[i].write_pos) {
      const double dt = static_cast<double>(now - prev[i].at_ns) / 1e9;
      if (dt > 0.0) rate = static_cast<double>(wp - prev[i].write_pos) / dt;
    }
    prev[i] = sample{wp, now, true};

    if (quiet) continue;

    if (opt.csv) {
      std::string cname(e.name, e.name_len);
      std::printf("topic,%llu,%s,%u,%d,%llu,%.3f,%llu,%llu\n",
                  static_cast<unsigned long long>(now), cname.c_str(), pid,
                  alive ? 1 : 0, static_cast<unsigned long long>(wp),
                  rate < 0.0 ? 0.0 : rate,
                  static_cast<unsigned long long>(cap),
                  static_cast<unsigned long long>(ov));
      continue;
    }

    char rate_s[16];
    if (rate < 0.0) std::snprintf(rate_s, sizeof rate_s, "-");
    else std::snprintf(rate_s, sizeof rate_s, "%.1f", rate);

    // A dead publisher is the single most useful thing this tool can show, so
    // it gets the loudest colour. "unowned" means the reaper already cleaned up.
    const char* state_col = alive ? g_c.green : (pid == 0 ? g_c.yellow : g_c.red);
    const char* state_txt = alive ? "alive" : (pid == 0 ? "unowned" : "DEAD");

    std::string name(e.name, e.name_len);
    if (name.size() > 22) name = name.substr(0, 21) + "~";

    std::printf("%-22s %8u %s%8s%s %10llu %9s %7llu %s%9llu%s %18llx\n",
                name.c_str(), pid, state_col, state_txt, g_c.reset,
                static_cast<unsigned long long>(wp), rate_s,
                static_cast<unsigned long long>(cap),
                ov ? g_c.yellow : "", static_cast<unsigned long long>(ov),
                ov ? g_c.reset : "",
                static_cast<unsigned long long>(e.layout_hash));
  }

  if (quiet) return;

  if (opt.csv) {
    ls::arena& car = b.allocator();
    for (std::uint32_t i = 0; i < car.class_count(); ++i) {
      const ls::pool_desc& p = car.pool(i);
      std::printf("arena,%llu,%u,%llu,%llu,%llu\n",
                  static_cast<unsigned long long>(now), i,
                  static_cast<unsigned long long>(p.block_size),
                  static_cast<unsigned long long>(
                      p.live_blocks.load(std::memory_order_relaxed)),
                  static_cast<unsigned long long>(p.block_count));
    }
    std::fflush(stdout);
    return;
  }

  if (shown == 0) std::printf("%s  (no topics yet)%s\n", g_c.dim, g_c.reset);

  ls::arena& ar = b.allocator();
  std::printf("\n%s%-6s %10s %10s %8s  %s%s\n", g_c.dim, "POOL", "BLOCK", "IN USE",
              "OF", "USAGE", g_c.reset);
  std::uint64_t live_total = 0, block_total = 0;
  for (std::uint32_t i = 0; i < ar.class_count(); ++i) {
    const ls::pool_desc& p = ar.pool(i);
    const std::uint64_t live = p.live_blocks.load(std::memory_order_relaxed);
    live_total += live;
    block_total += p.block_count;
    // Flag a pool that is three-quarters gone: allocation failure at startup is
    // the usual symptom of a pool sized too small, and this is where it shows.
    const bool tight = p.block_count && (live * 4 >= p.block_count * 3);
    std::printf("%-6u %10s %s%10llu%s %8llu  %s\n", i,
                human_bytes(p.block_size).c_str(),
                tight ? g_c.yellow : "", static_cast<unsigned long long>(live),
                tight ? g_c.reset : "",
                static_cast<unsigned long long>(p.block_count),
                bar(live, p.block_count).c_str());
  }
  std::printf("%s%-6s %10s %10llu %8llu%s\n", g_c.dim, "total", "",
              static_cast<unsigned long long>(live_total),
              static_cast<unsigned long long>(block_total), g_c.reset);
}

}  // namespace

int main(int argc, char** argv) {
  options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
    if (a == "--help" || a == "-h") { usage(); return 0; }
    else if (a == "--once") opt.once = true;
    else if (a == "--no-colour" || a == "--no-color") opt.no_colour = true;
    else if (a == "--csv") { opt.csv = true; opt.no_colour = true; }
    else if (a == "--bus") { const char* v = next(); if (!v) return 1; opt.bus = v; }
    else if (a == "--interval") { const char* v = next(); if (!v) return 1;
                                  opt.interval_ms = std::atoi(v); }
    else { std::fprintf(stderr, "unknown option: %s\n", a.c_str()); usage(); return 1; }
  }
  if (opt.interval_ms < 50) opt.interval_ms = 50;
  if (opt.no_colour) strip_colour();
  enable_ansi();

  ls::bus b;
  try {
    b = ls::bus::open(opt.bus);
  } catch (const std::exception& ex) {
    std::fprintf(stderr, "lockstep_top: cannot attach to '%s': %s\n",
                 opt.bus.c_str(), ex.what());
    std::fprintf(stderr,
                 "  Is a publisher running? The bus is created by whichever process\n"
                 "  starts first (elegoo_bridge, five_node_demo, ...).\n");
    return 1;
  }

  std::vector<sample> prev(b.topics().capacity());
  const std::uint64_t started = ls::now_ns();

  // A rate is a difference between two samples, so a single-shot run still has
  // to take two: one silent priming pass, then the real frame one interval
  // later. Without this --once could only ever print "-", which would make it
  // useless for the scripted checks it exists for.
  if (opt.once) {
    render(b, prev, opt, started, /*quiet=*/true);
    ls::sleep_us(static_cast<unsigned>(opt.interval_ms) * 1000u);
    render(b, prev, opt, started, /*quiet=*/false);
    return 0;
  }

  for (;;) {
    render(b, prev, opt, started, /*quiet=*/false);
    std::printf("\n%srefresh %dms   ctrl-c to quit%s\n", g_c.dim, opt.interval_ms,
                g_c.reset);
    std::fflush(stdout);
    ls::sleep_us(static_cast<unsigned>(opt.interval_ms) * 1000u);
  }
}
