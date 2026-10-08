// Tests for the robot bridge, with no robot.
//
// Everything here runs on a desk with the car still in its box. Two halves:
//
//   1. the protocol encoders and the reply parser are pure functions, so they
//      are pinned exactly -- if an edit changes a command string, this notices;
//   2. the socket layer is driven against a FAKE ROBOT on loopback, which
//      speaks the replies the real firmware is documented to speak. That covers
//      connect, framing, timeout and disconnection.
//
// What this cannot cover, and does not pretend to: whether the real firmware
// agrees with robot/elegoo/protocol.hpp. Only the robot can answer that, which
// is what `elegoo_bridge --probe` is for.
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>

#include "avi.hpp"
#include "lockstep/core/clock.hpp"
#include "lockstep/net/tcp.hpp"
#include "lockstep/replay/journal.hpp"
#include "protocol.hpp"
#include "robot_msgs.hpp"
#include "support/check.hpp"

namespace {

using namespace ls::elegoo;

// ---------------------------------------------------------------------------
// Encoding
// ---------------------------------------------------------------------------

void commands_encode_exactly() {
  LS_CHECK(encode_stop() == std::string("{\"H\":\"ls\",\"N\":100}"));
  LS_CHECK(encode_ultrasonic() ==
           std::string("{\"H\":\"ls\",\"N\":21,\"D1\":2}"));
  LS_CHECK(encode_move(direction::forward, 150) ==
           std::string("{\"H\":\"ls\",\"N\":3,\"D1\":3,\"D2\":150}"));
  LS_CHECK(encode_move(direction::back, 150) ==
           std::string("{\"H\":\"ls\",\"N\":3,\"D1\":4,\"D2\":150}"));
  LS_CHECK(encode_servo(90) ==
           std::string("{\"H\":\"ls\",\"N\":5,\"D1\":1,\"D2\":90}"));
}

// The "H" tag is not decoration. A query without it is discarded by the
// firmware in silence -- that one missing field is why the bridge talked to a
// working car for an entire session and received nothing back.
void every_command_carries_the_handshake_tag() {
  LS_CHECK(encode_stop().find("\"H\":\"ls\"") != std::string::npos);
  LS_CHECK(encode_ultrasonic().find("\"H\":\"ls\"") != std::string::npos);
  LS_CHECK(encode_line_track().find("\"H\":\"ls\"") != std::string::npos);
  LS_CHECK(encode_left(150).find("\"H\":\"ls\"") != std::string::npos);
  LS_CHECK(encode_right(150).find("\"H\":\"ls\"") != std::string::npos);
  LS_CHECK(encode_move(direction::forward, 150).find("\"H\":\"ls\"") !=
           std::string::npos);
  LS_CHECK(encode_servo(90).find("\"H\":\"ls\"") != std::string::npos);
}

// The keepalive is a bare frame, not a numbered command, and the ESP32 hangs up
// after three unanswered ones.
void the_keepalive_is_a_bare_frame() {
  LS_CHECK(encode_heartbeat() == std::string("{Heartbeat}"));
  LS_CHECK(is_heartbeat("{Heartbeat}"));
  LS_CHECK(!is_heartbeat("{ls_52}"));
  LS_CHECK(!is_heartbeat("{ok}"));
}

// Reverse is a sign on the speed, encoded as a direction field plus magnitude.
// Getting this backwards would drive the car the wrong way, which is exactly
// the class of bug worth pinning.
void reverse_is_encoded_as_direction_zero() {
  const std::string fwd = encode_left(150);
  const std::string rev = encode_left(-150);
  LS_CHECK(fwd == std::string("{\"H\":\"ls\",\"N\":1,\"D1\":1,\"D2\":150,\"D3\":1}"));
  LS_CHECK(rev == std::string("{\"H\":\"ls\",\"N\":1,\"D1\":1,\"D2\":150,\"D3\":0}"));
  LS_CHECK(fwd != rev);
}

void left_and_right_select_different_motors() {
  LS_CHECK(encode_left(120) != encode_right(120));
  LS_CHECK(encode_left(120).find("\"D1\":1") != std::string::npos);
  LS_CHECK(encode_right(120).find("\"D1\":2") != std::string::npos);
}

// Below the deadband the motors stall rather than turning slowly, so a small
// request must become a real zero. Otherwise a controller asking for a gentle
// crawl gets silence and looks broken.
void speeds_below_the_deadband_become_zero() {
  const profile p;
  LS_CHECK_EQ(apply_deadband(0, p), 0);
  LS_CHECK_EQ(apply_deadband(p.speed_deadband - 1, p), 0);
  LS_CHECK_EQ(apply_deadband(-(p.speed_deadband - 1), p), 0);
  LS_CHECK_EQ(apply_deadband(p.speed_deadband, p), p.speed_deadband);
  LS_CHECK_EQ(apply_deadband(200, p), 200);
  LS_CHECK_EQ(apply_deadband(-200, p), -200);

  LS_CHECK(encode_left(10).find("\"D2\":0") != std::string::npos);
}

void speeds_are_clamped_to_the_hardware_range() {
  const profile p;
  LS_CHECK_EQ(clamp_speed(99999, p), p.speed_max);
  LS_CHECK_EQ(clamp_speed(-99999, p), -p.speed_max);
  LS_CHECK(encode_left(99999).find("\"D2\":255") != std::string::npos);
}

// ---------------------------------------------------------------------------
// Reply parsing. The firmware is loose about format, so the parser has to be
// permissive -- these are the shapes it is documented to produce.
// ---------------------------------------------------------------------------
void replies_parse() {
  const reply ok = parse_reply("{ok}");
  LS_CHECK(ok.affirmative);
  LS_CHECK(!ok.has_value);

  const reply t = parse_reply("{true}");
  LS_CHECK(t.affirmative);

  const reply f = parse_reply("{false}");
  LS_CHECK(!f.affirmative);

  const reply dist = parse_reply("{42}");
  LS_CHECK(dist.has_value);
  LS_CHECK_EQ(dist.value, 42L);
  LS_CHECK(dist.affirmative);

  const reply underscored = parse_reply("_137_");
  LS_CHECK(underscored.has_value);
  LS_CHECK_EQ(underscored.value, 137L);

  const reply crlf = parse_reply("{99}\r\n");
  LS_CHECK_EQ(crlf.value, 99L);

  // The shape the real car actually answers with: {<tag>_<payload>}.
  const reply tagged = parse_reply("{ls_52}");
  LS_CHECK(tagged.has_value);
  LS_CHECK_EQ(tagged.value, 52L);
  LS_CHECK(tagged.affirmative);

  const reply tagged_ok = parse_reply("{ls_ok}");
  LS_CHECK(tagged_ok.affirmative);
  LS_CHECK(!tagged_ok.has_value);

  const reply tagged_false = parse_reply("{ls_false}");
  LS_CHECK(!tagged_false.affirmative);

  // The payload is the LAST underscore-separated field, not the first integer
  // in the string. A tag with a digit in it would otherwise be read as the
  // distance, and the car would brake at a number that came from its own tag.
  const reply digit_tag = parse_reply("{ls2_52}");
  LS_CHECK(digit_tag.has_value);
  LS_CHECK_EQ(digit_tag.value, 52L);

  const reply empty = parse_reply("");
  LS_CHECK(!empty.affirmative);
  LS_CHECK(!empty.has_value);

  // The raw text survives untouched, because --probe shows it to a human who is
  // trying to work out what their firmware does.
  LS_CHECK(parse_reply("{weird}").raw == std::string("{weird}"));
}

// A lost echo reads as 0 or as a number far past the sensor's range. Feeding
// either to a controller makes the car swerve at nothing.
void implausible_ranges_are_rejected() {
  LS_CHECK(!range_is_plausible(0));
  LS_CHECK(!range_is_plausible(1));
  LS_CHECK(!range_is_plausible(5000));
  LS_CHECK(!range_is_plausible(-3));
  LS_CHECK(range_is_plausible(2));
  LS_CHECK(range_is_plausible(50));
  LS_CHECK(range_is_plausible(400));
}

// ---------------------------------------------------------------------------
// The socket layer, against a fake robot.
// ---------------------------------------------------------------------------

// Speaks just enough Elegoo to exercise the bridge's link handling: brace
// framed with NO trailing newline, exactly like the real firmware, answering
// range queries with a tagged number and everything else with {ok}.
struct fake_robot {
  ls::net::tcp_listener listener;
  std::thread thread;
  std::atomic<bool> running{false};
  std::atomic<int> commands_seen{0};
  long range_to_report = 57;

  bool start() {
    if (!listener.listen_loopback()) return false;
    running.store(true);
    thread = std::thread([this] {
      ls::net::tcp_stream c = listener.accept(3000);
      if (!c.valid()) return;
      std::string line;
      while (running.load() && c.recv_frame(line, 1000)) {
        commands_seen.fetch_add(1);
        const std::string resp =
            line.find("\"N\":21") != std::string::npos
                ? "{ls_" + std::to_string(range_to_report) + "}"
                : std::string("{ok}");
        if (!c.send_all(resp)) break;
      }
    });
    return true;
  }

  void stop() {
    running.store(false);
    if (thread.joinable()) thread.join();
  }

  std::uint16_t port() const { return listener.port(); }
};

void the_link_talks_to_a_fake_robot() {
  fake_robot robot;
  LS_CHECK(robot.start());

  std::string err;
  ls::net::tcp_stream c =
      ls::net::tcp_stream::connect("127.0.0.1", robot.port(), 2000, &err);
  LS_CHECK(c.valid());
  if (!c.valid()) {
    std::fprintf(stderr, "  connect failed: %s\n", err.c_str());
    robot.stop();
    return;
  }

  // A range query round-trips and parses to the value the robot reported.
  // Sent with no trailing newline, and read back by brace framing -- waiting
  // for a '\n' here is what made a correct reply look like a timeout.
  LS_CHECK(c.send_all(encode_ultrasonic()));
  std::string line;
  LS_CHECK(c.recv_frame(line, 2000));
  const reply r = parse_reply(line);
  LS_CHECK(r.has_value);
  LS_CHECK_EQ(r.value, 57L);
  LS_CHECK(range_is_plausible(r.value));

  // A drive command is acknowledged.
  LS_CHECK(c.send_all(encode_left(150)));
  LS_CHECK(c.recv_frame(line, 2000));
  LS_CHECK(parse_reply(line).affirmative);

  LS_CHECK(robot.commands_seen.load() >= 2);
  c.close();
  robot.stop();
}

// Connecting to a port nobody is listening on must fail promptly rather than
// hanging the node for the OS default, which is around two minutes.
void connecting_to_nothing_fails_fast() {
  const std::uint64_t t0 = ls::now_ns();
  std::string err;
  // Port 1 on loopback: reserved, and nothing will be bound there.
  ls::net::tcp_stream c = ls::net::tcp_stream::connect("127.0.0.1", 1, 500, &err);
  const std::uint64_t took_ms = (ls::now_ns() - t0) / 1000000u;

  LS_CHECK(!c.valid());
  LS_CHECK(!err.empty());
  LS_CHECK(took_ms < 3000);  // generous, but nowhere near the OS default
}

// If the robot goes away mid-session the bridge has to notice, not block.
void a_dead_robot_is_detected() {
  fake_robot robot;
  LS_CHECK(robot.start());

  std::string err;
  ls::net::tcp_stream c =
      ls::net::tcp_stream::connect("127.0.0.1", robot.port(), 2000, &err);
  LS_CHECK(c.valid());

  robot.stop();  // the car is switched off

  // Reads must stop returning data rather than hanging forever.
  std::string line;
  bool got = c.recv_frame(line, 300);
  LS_CHECK(!got);
}

// ---------------------------------------------------------------------------
// The message types
// ---------------------------------------------------------------------------
void robot_messages_are_replay_safe() {
  // Padding is indeterminate and a replay hash covers the whole slot, so these
  // being padding-free is a correctness property, not a cosmetic one.
  LS_CHECK(!ls::message_traits<RobotRange>::has_padding);
  LS_CHECK(!ls::message_traits<RobotDrive>::has_padding);
  LS_CHECK(!ls::message_traits<RobotStatus>::has_padding);

  LS_CHECK(ls::message_traits<RobotRange>::declared);
  LS_CHECK(ls::message_traits<RobotDrive>::declared);
  LS_CHECK(ls::message_traits<RobotStatus>::declared);

  // Distinct types must not be confusable at connect time.
  LS_CHECK(ls::message_traits<RobotRange>::layout_hash !=
           ls::message_traits<RobotDrive>::layout_hash);

  // A camera frame rides in the 64 KB pool slot exactly. If this ever stops
  // being true the publisher fails at create() on a real drive, which is a
  // miserable place to find out.
  LS_CHECK(!ls::message_traits<RobotFrame>::has_padding);
  LS_CHECK(sizeof(RobotFrame) == 65536);
}

// ---------------------------------------------------------------------------
// Video
//
// The AVI muxer is bytes-in-the-right-place code with no way to eyeball it: a
// wrong offset produces a file that exists, has the right size, and plays as
// noise. So the structure is read back and checked field by field.
// ---------------------------------------------------------------------------

// A byte sequence with the markers jpeg_size() reads, and nothing else. Enough
// to exercise the parser and the muxer without embedding a real photograph in
// the test suite.
std::string fake_jpeg(std::uint16_t w, std::uint16_t h, std::size_t pad = 8) {
  std::string s;
  s.push_back(static_cast<char>(0xFF));
  s.push_back(static_cast<char>(0xD8));            // SOI
  s.push_back(static_cast<char>(0xFF));
  s.push_back(static_cast<char>(0xC0));            // SOF0
  s.push_back(0x00);
  s.push_back(0x11);                               // length 17
  s.push_back(0x08);                               // precision
  s.push_back(static_cast<char>((h >> 8) & 0xFF));
  s.push_back(static_cast<char>(h & 0xFF));
  s.push_back(static_cast<char>((w >> 8) & 0xFF));
  s.push_back(static_cast<char>(w & 0xFF));
  s.append(pad, '\x55');                           // stand-in for scan data
  s.push_back(static_cast<char>(0xFF));
  s.push_back(static_cast<char>(0xD9));            // EOI
  return s;
}

std::uint32_t le32(const std::string& b, std::size_t at) {
  return static_cast<std::uint32_t>(static_cast<std::uint8_t>(b[at])) |
         (static_cast<std::uint32_t>(static_cast<std::uint8_t>(b[at + 1])) << 8) |
         (static_cast<std::uint32_t>(static_cast<std::uint8_t>(b[at + 2])) << 16) |
         (static_cast<std::uint32_t>(static_cast<std::uint8_t>(b[at + 3])) << 24);
}

std::string read_file(const std::string& path) {
  std::string out;
  std::FILE* f = nullptr;
#if defined(_WIN32)
  if (fopen_s(&f, path.c_str(), "rb") != 0) f = nullptr;
#else
  f = std::fopen(path.c_str(), "rb");
#endif
  if (f == nullptr) return out;
  char buf[4096];
  std::size_t n = 0;
  while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) out.append(buf, n);
  std::fclose(f);
  return out;
}

void jpeg_dimensions_are_read_from_the_sof_marker() {
  std::uint16_t w = 0, h = 0;
  const std::string j = fake_jpeg(640, 480);
  LS_CHECK(ls::avi::jpeg_size(reinterpret_cast<const std::uint8_t*>(j.data()),
                              j.size(), w, h));
  LS_CHECK(w == 640);
  LS_CHECK(h == 480);

  // Anything that is not a JPEG must be refused rather than guessed at. An
  // ESP32 serving an HTML error page with a 200 produces exactly this.
  const std::string html = "<html>not a frame</html>";
  LS_CHECK(!ls::avi::jpeg_size(
      reinterpret_cast<const std::uint8_t*>(html.data()), html.size(), w, h));

  // Truncated after the SOI: no SOF marker to find.
  const std::uint8_t soi_only[2] = {0xFF, 0xD8};
  LS_CHECK(!ls::avi::jpeg_size(soi_only, sizeof soi_only, w, h));
}

void the_avi_file_is_structurally_valid() {
  const std::string path = "test_avi_out.avi";
  // One of each parity, so the word-alignment pad byte is exercised either way.
  const std::string f1 = fake_jpeg(320, 240, 9);
  const std::string f2 = fake_jpeg(320, 240, 8);
  LS_CHECK((f1.size() & 1u) != (f2.size() & 1u));

  // A chunk is 8 bytes of header, the payload, then a pad byte if the payload
  // length is odd. The pad is not counted in the length.
  const auto chunk_span = [](std::size_t payload) {
    return 8 + payload + (payload & 1u);
  };

  {
    ls::avi::mjpeg_writer w;
    LS_CHECK(w.open(path, 4));
    LS_CHECK(w.add_frame(reinterpret_cast<const std::uint8_t*>(f1.data()),
                         static_cast<std::uint32_t>(f1.size())));
    LS_CHECK(w.add_frame(reinterpret_cast<const std::uint8_t*>(f2.data()),
                         static_cast<std::uint32_t>(f2.size())));

    // A frame whose dimensions differ from the first is refused: AVI declares
    // one frame size for the whole stream.
    const std::string odd = fake_jpeg(160, 120);
    LS_CHECK(!w.add_frame(reinterpret_cast<const std::uint8_t*>(odd.data()),
                          static_cast<std::uint32_t>(odd.size())));
    LS_CHECK(w.frames() == 2);
    LS_CHECK(w.rejected() == 1);
    LS_CHECK(w.close());
  }

  const std::string b = read_file(path);
  LS_CHECK(b.size() > 224);
  LS_CHECK(b.compare(0, 4, "RIFF") == 0);
  LS_CHECK(b.compare(8, 4, "AVI ") == 0);
  LS_CHECK(b.compare(12, 4, "LIST") == 0);
  LS_CHECK(b.compare(20, 4, "hdrl") == 0);
  LS_CHECK(b.compare(24, 4, "avih") == 0);

  // RIFF size counts everything after the size field itself.
  LS_CHECK(le32(b, 4) == b.size() - 8);

  // The fixed layout the patch offsets in avi.hpp depend on.
  LS_CHECK(le32(b, 16) == 192);                  // hdrl list size
  LS_CHECK(b.compare(100, 4, "strh") == 0);
  LS_CHECK(b.compare(108, 4, "vids") == 0);
  LS_CHECK(b.compare(112, 4, "MJPG") == 0);
  LS_CHECK(b.compare(164, 4, "strf") == 0);
  LS_CHECK(b.compare(212, 4, "LIST") == 0);
  LS_CHECK(b.compare(220, 4, "movi") == 0);

  // Frame count, dimensions and rate, in both headers that carry them.
  LS_CHECK(le32(b, 48) == 2);                    // avih dwTotalFrames
  LS_CHECK(le32(b, 64) == 320);                  // avih dwWidth
  LS_CHECK(le32(b, 68) == 240);                  // avih dwHeight
  LS_CHECK(le32(b, 32) == 250000u);              // 4 fps -> 250000 us/frame
  LS_CHECK(le32(b, 140) == 2);                   // strh dwLength
  LS_CHECK(le32(b, 176) == 320);                 // biWidth
  LS_CHECK(le32(b, 180) == 240);                 // biHeight
  LS_CHECK(b.compare(188, 4, "MJPG") == 0);      // biCompression

  // First frame chunk sits immediately after the 'movi' FourCC.
  LS_CHECK(b.compare(224, 4, "00dc") == 0);
  LS_CHECK(le32(b, 228) == f1.size());

  // Every chunk must start on an even offset, which is what the pad byte is
  // for. A reader that gets this wrong sees the pad as the start of a FourCC.
  const std::size_t second = 224 + chunk_span(f1.size());
  LS_CHECK((second & 1u) == 0u);
  LS_CHECK(b.compare(second, 4, "00dc") == 0);
  LS_CHECK(le32(b, second + 4) == f2.size());

  // idx1 follows the last frame, 16 bytes per entry, offsets relative to the
  // 'movi' FourCC -- so the first is 4, not 0 and not an absolute position.
  const std::size_t idx = second + chunk_span(f2.size());
  LS_CHECK(b.compare(idx, 4, "idx1") == 0);
  LS_CHECK(le32(b, idx + 4) == 32);
  LS_CHECK(b.compare(idx + 8, 4, "00dc") == 0);
  LS_CHECK(le32(b, idx + 12) == 0x10u);          // AVIIF_KEYFRAME
  LS_CHECK(le32(b, idx + 16) == 4);
  LS_CHECK(le32(b, idx + 20) == f1.size());
  LS_CHECK(le32(b, idx + 32) == static_cast<std::uint32_t>(second - 220));

  // movi list size spans the FourCC and every frame chunk.
  LS_CHECK(le32(b, 216) == idx - 220);

  std::remove(path.c_str());
}

// ---------------------------------------------------------------------------
// Journalling frames alongside readings
//
// deterministic_clock, in replay, walks the journal forward with the SAME
// cursor the reader uses, discarding records until it finds a clock record. So
// where a frame is appended relative to the decision's clock read decides
// whether the frame survives replay at all.
//
// This pins both directions, because the failure mode of getting it wrong is
// silent: the video simply comes out short, and nothing reports an error.
// ---------------------------------------------------------------------------
constexpr std::uint32_t kRangeCb = 0;
constexpr std::uint32_t kFrameCb = 1;

// Walks a journal the way avoider's replay does -- one reading, one clock read,
// frames skipped past -- and returns how many frames it saw.
int frames_visible_on_replay(const std::string& path) {
  ls::journal j = ls::journal::open_read(path);
  ls::deterministic_clock clk(ls::run_mode::replay, &j);
  int frames = 0;
  ls::record rec;
  while (j.next(rec)) {
    if (rec.kind != ls::record_kind::message) continue;
    if (rec.callback == kFrameCb) { ++frames; continue; }
    (void)clk.now_ns();  // the decision's clock read, as the live run made it
  }
  return frames;
}

void a_frame_journalled_after_the_clock_read_survives_replay() {
  const std::string path = "test_frame_order_ok.jrnl";
  const std::string jpeg = fake_jpeg(64, 48);
  {
    ls::journal j = ls::journal::create(path, 1u << 16);
    RobotRange r{};
    r.valid = 1;
    r.distance_cm = 50.0f;
    for (int i = 0; i < 3; ++i) {
      j.append(ls::record_kind::message, kRangeCb, static_cast<std::uint64_t>(i),
               0, &r, sizeof r);
      j.append(ls::record_kind::clock, 0, static_cast<std::uint64_t>(i), 1000, nullptr, 0);
      j.append(ls::record_kind::message, kFrameCb, static_cast<std::uint64_t>(i),
               0, jpeg.data(), static_cast<std::uint32_t>(jpeg.size()));
    }
  }
  LS_CHECK(frames_visible_on_replay(path) == 3);
  std::remove(path.c_str());
}

void a_frame_journalled_before_the_clock_read_is_lost() {
  // Not a wish, a warning: this is what the ordering constraint protects
  // against, and it is here so that anyone who moves the append in avoider.cpp
  // sees a red test instead of a video that is quietly missing every frame.
  const std::string path = "test_frame_order_bad.jrnl";
  const std::string jpeg = fake_jpeg(64, 48);
  {
    ls::journal j = ls::journal::create(path, 1u << 16);
    RobotRange r{};
    r.valid = 1;
    r.distance_cm = 50.0f;
    for (int i = 0; i < 3; ++i) {
      j.append(ls::record_kind::message, kRangeCb, static_cast<std::uint64_t>(i),
               0, &r, sizeof r);
      j.append(ls::record_kind::message, kFrameCb, static_cast<std::uint64_t>(i),
               0, jpeg.data(), static_cast<std::uint32_t>(jpeg.size()));
      j.append(ls::record_kind::clock, 0, static_cast<std::uint64_t>(i), 1000, nullptr, 0);
    }
  }
  LS_CHECK(frames_visible_on_replay(path) == 0);
  std::remove(path.c_str());
}

// The end to end path: journalled frames come back out as a playable video.
void a_journal_replays_into_a_video() {
  const std::string jpath = "test_video_roundtrip.jrnl";
  const std::string vpath = "test_video_roundtrip.avi";
  const std::string jpeg = fake_jpeg(160, 120);
  {
    ls::journal j = ls::journal::create(jpath, 1u << 16);
    RobotRange r{};
    r.valid = 1;
    r.distance_cm = 40.0f;
    for (int i = 0; i < 5; ++i) {
      j.append(ls::record_kind::message, kRangeCb, static_cast<std::uint64_t>(i),
               0, &r, sizeof r);
      j.append(ls::record_kind::clock, 0, static_cast<std::uint64_t>(i), 1000, nullptr, 0);
      j.append(ls::record_kind::message, kFrameCb, static_cast<std::uint64_t>(i),
               0, jpeg.data(), static_cast<std::uint32_t>(jpeg.size()));
    }
  }

  {
    ls::journal j = ls::journal::open_read(jpath);
    ls::deterministic_clock clk(ls::run_mode::replay, &j);
    ls::avi::mjpeg_writer w;
    LS_CHECK(w.open(vpath, 4));
    ls::record rec;
    while (j.next(rec)) {
      if (rec.kind != ls::record_kind::message) continue;
      if (rec.callback == kFrameCb) {
        w.add_frame(static_cast<const std::uint8_t*>(rec.payload), rec.bytes);
        continue;
      }
      (void)clk.now_ns();
    }
    LS_CHECK(w.frames() == 5);
    LS_CHECK(w.close());
  }

  const std::string b = read_file(vpath);
  LS_CHECK(b.compare(0, 4, "RIFF") == 0);
  LS_CHECK(le32(b, 48) == 5);      // five frames made it through
  LS_CHECK(le32(b, 64) == 160);
  LS_CHECK(le32(b, 68) == 120);

  std::remove(jpath.c_str());
  std::remove(vpath.c_str());
}

// The rate is only measurable once the frames are in, so it is patched at
// close. Both places AVI states it have to change together -- a player that
// reads the one left stale plays the drive at the wrong speed.
void the_playback_rate_can_be_corrected_at_close() {
  const std::string path = "test_avi_fps.avi";
  const std::string j = fake_jpeg(320, 240);
  {
    ls::avi::mjpeg_writer w;
    LS_CHECK(w.open(path, 4));           // opened claiming 4 fps
    LS_CHECK(w.add_frame(reinterpret_cast<const std::uint8_t*>(j.data()),
                         static_cast<std::uint32_t>(j.size())));
    w.set_fps(12);                       // measured higher
    LS_CHECK(w.close());
  }
  const std::string b = read_file(path);
  LS_CHECK(le32(b, 32) == 1000000u / 12u);   // avih dwMicroSecPerFrame
  LS_CHECK(le32(b, 108 + 24) == 12u);        // strh dwRate
  LS_CHECK(le32(b, 108 + 20) == 1u);         // strh dwScale, rate is per second
  std::remove(path.c_str());
}

void a_video_with_no_readable_frames_writes_nothing() {
  const std::string path = "test_avi_empty.avi";
  ls::avi::mjpeg_writer w;
  LS_CHECK(w.open(path, 4));
  const std::string junk = "not a jpeg at all";
  LS_CHECK(!w.add_frame(reinterpret_cast<const std::uint8_t*>(junk.data()),
                        static_cast<std::uint32_t>(junk.size())));
  LS_CHECK(w.frames() == 0);
  LS_CHECK(w.close());
  // The header is still on disk -- it is the caller's job to discard a
  // frameless file, which video_sink::close() does.
  LS_CHECK(read_file(path).size() == 224 + 8);
  std::remove(path.c_str());
}

}  // namespace

int main() {
  commands_encode_exactly();
  every_command_carries_the_handshake_tag();
  the_keepalive_is_a_bare_frame();
  reverse_is_encoded_as_direction_zero();
  left_and_right_select_different_motors();
  speeds_below_the_deadband_become_zero();
  speeds_are_clamped_to_the_hardware_range();
  replies_parse();
  implausible_ranges_are_rejected();
  robot_messages_are_replay_safe();
  jpeg_dimensions_are_read_from_the_sof_marker();
  the_avi_file_is_structurally_valid();
  the_playback_rate_can_be_corrected_at_close();
  a_video_with_no_readable_frames_writes_nothing();
  a_frame_journalled_after_the_clock_read_survives_replay();
  a_frame_journalled_before_the_clock_read_is_lost();
  a_journal_replays_into_a_video();
  the_link_talks_to_a_fake_robot();
  connecting_to_nothing_fails_fast();
  a_dead_robot_is_detected();
  return ls::test::summary("elegoo");
}
