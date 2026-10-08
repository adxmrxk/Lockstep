// robot/elegoo/avi.hpp : write a pile of JPEGs out as a playable video file.
//
// The car's camera already hands us JPEG frames, and Motion JPEG is JPEG frames
// in a container. So there is no encoding to do here at all -- this is a muxer,
// about 200 lines of RIFF bookkeeping, and the result opens in VLC, Windows
// Media Player, QuickTime and anything ffmpeg touches.
//
// WHY NOT JUST SHELL OUT TO FFMPEG
//
// Because then the feature works on machines that have ffmpeg and silently does
// not on machines that do not, which is the worse failure: you find out after
// the drive you wanted to watch. A muxer that ships with the binary always
// works. The cost is that the output is MJPEG-in-AVI rather than H.264 -- bigger
// files, no interframe compression -- which for a 30 second robot drive at 4 fps
// is a few megabytes and nobody cares.
//
// WHY NOT MP4
//
// MP4 essentially requires H.264, and an H.264 encoder is not 200 lines. AVI
// will hold MJPEG with no transcoding, so AVI it is. `ffmpeg -i run.avi run.mp4`
// converts it for anyone who wants that, losslessly as far as the frames go.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace ls::avi {

// Width and height out of a JPEG's SOF marker.
//
// Needed because the AVI header must state the frame size up front, and the
// camera does not tell us what it decided to send. Returns false on anything
// that is not a JPEG we can read a frame size out of, which the caller treats
// as "not a usable frame" rather than guessing a size and writing a file that
// plays as garbage.
inline bool jpeg_size(const std::uint8_t* p, std::size_t n, std::uint16_t& w,
                      std::uint16_t& h) {
  if (n < 4 || p[0] != 0xFF || p[1] != 0xD8) return false;  // not SOI
  std::size_t i = 2;
  while (i + 3 < n) {
    if (p[i] != 0xFF) { ++i; continue; }        // resync past fill bytes
    const std::uint8_t marker = p[i + 1];
    if (marker == 0xFF) { ++i; continue; }
    if (marker == 0xD8 || marker == 0xD9 || (marker >= 0xD0 && marker <= 0xD7)) {
      i += 2;                                    // standalone marker, no length
      continue;
    }
    const std::size_t len =
        (static_cast<std::size_t>(p[i + 2]) << 8) | p[i + 3];
    // SOF0 baseline, SOF1 extended, SOF2 progressive, SOF3 lossless. The
    // frame size sits at the same offset in all of them.
    if (marker >= 0xC0 && marker <= 0xC3) {
      if (i + 9 >= n) return false;
      h = static_cast<std::uint16_t>((p[i + 5] << 8) | p[i + 6]);
      w = static_cast<std::uint16_t>((p[i + 7] << 8) | p[i + 8]);
      return w != 0 && h != 0;
    }
    if (len < 2) return false;
    i += 2 + len;
  }
  return false;
}

// Motion JPEG in an AVI container.
//
// Three things can only be written once the last frame is in -- the total frame
// count, the RIFF and movi sizes, and the index -- so the header goes down with
// placeholders, every frame's offset is remembered, and close() seeks back and
// patches. That means an abandoned file (killed process, full disk) is NOT
// playable, which is the one real limitation: the text transcript is the thing
// that survives a crash, and this is the thing you watch afterwards.
class mjpeg_writer {
 public:
  mjpeg_writer() = default;
  mjpeg_writer(const mjpeg_writer&) = delete;
  mjpeg_writer& operator=(const mjpeg_writer&) = delete;
  ~mjpeg_writer() { close(); }

  // `fps` is what the file claims for playback. Pass the rate frames were
  // actually captured at, so the video runs at the speed the drive happened.
  bool open(const std::string& path, int fps) {
    close();
    if (fps < 1) fps = 1;
    fps_ = fps;
    path_ = path;
#if defined(_WIN32)
    if (fopen_s(&f_, path.c_str(), "wb+") != 0) f_ = nullptr;
#else
    f_ = std::fopen(path.c_str(), "wb+");
#endif
    if (f_ == nullptr) { path_.clear(); return false; }
    index_.clear();
    w_ = h_ = 0;
    // Placeholder header. Patched by close() once the real numbers are known.
    return write_header();
  }

  bool is_open() const { return f_ != nullptr; }
  const std::string& path() const { return path_; }
  std::size_t frames() const { return index_.size(); }

  // Correct the playback rate before closing.
  //
  // Worth having because the rate is only guessable when the file is opened and
  // measurable once the frames are in. The camera is pulled on a best-effort
  // loop against a WiFi link, so what it actually delivered is never exactly
  // what was asked for, and a video tagged with the requested rate plays at the
  // wrong speed -- the drive looks faster or slower than it happened.
  void set_fps(int fps) { if (fps >= 1) fps_ = fps; }

  // Appends one JPEG. The first frame fixes the video's dimensions; a later
  // frame of a different size is dropped rather than written, because AVI has
  // one frame size for the whole stream and a mismatched frame renders as
  // garbage in some players and crashes others.
  bool add_frame(const std::uint8_t* jpeg, std::uint32_t bytes) {
    if (f_ == nullptr || jpeg == nullptr || bytes < 4) return false;

    std::uint16_t w = 0, h = 0;
    if (!jpeg_size(jpeg, bytes, w, h)) { ++rejected_; return false; }
    if (w_ == 0) { w_ = w; h_ = h; }
    else if (w != w_ || h != h_) { ++rejected_; return false; }

    // '00dc' = stream 0, compressed video. Chunks are word aligned, so an odd
    // length is followed by one pad byte that is not counted in the length.
    const long at = std::ftell(f_);
    if (at < 0) return false;
    if (!write_fourcc("00dc") || !write_u32(bytes)) return false;
    if (std::fwrite(jpeg, 1, bytes, f_) != bytes) return false;
    if ((bytes & 1u) != 0u && std::fputc(0, f_) == EOF) return false;

    index_.push_back(entry{static_cast<std::uint32_t>(at), bytes});
    return true;
  }

  // How many frames were handed over but not written, because they were not
  // readable JPEGs or changed size mid-stream. Surfaced so a run that produced
  // a short video says why instead of leaving you to wonder.
  std::uint64_t rejected() const { return rejected_; }

  // Writes the index and patches the header. Safe to call twice.
  bool close() {
    if (f_ == nullptr) return true;
    bool ok = write_index() && patch_header();
    std::fclose(f_);
    f_ = nullptr;
    return ok;
  }

 private:
  struct entry { std::uint32_t offset; std::uint32_t bytes; };

  bool write_fourcc(const char* s) {
    return std::fwrite(s, 1, 4, f_) == 4;
  }

  // AVI is little-endian regardless of host, so the bytes are placed by hand
  // rather than by writing the integer's memory.
  bool write_u32(std::uint32_t v) {
    const std::uint8_t b[4] = {
        static_cast<std::uint8_t>(v & 0xFFu),
        static_cast<std::uint8_t>((v >> 8) & 0xFFu),
        static_cast<std::uint8_t>((v >> 16) & 0xFFu),
        static_cast<std::uint8_t>((v >> 24) & 0xFFu)};
    return std::fwrite(b, 1, 4, f_) == 4;
  }

  bool write_u16(std::uint16_t v) {
    const std::uint8_t b[2] = {static_cast<std::uint8_t>(v & 0xFFu),
                               static_cast<std::uint8_t>((v >> 8) & 0xFFu)};
    return std::fwrite(b, 1, 2, f_) == 2;
  }

  bool patch_u32_at(long offset, std::uint32_t v) {
    if (std::fseek(f_, offset, SEEK_SET) != 0) return false;
    return write_u32(v);
  }

  // RIFF('AVI ' LIST('hdrl' 'avih' LIST('strl' 'strh' 'strf')) LIST('movi' ...
  //
  // Fixed layout, so the offsets of the fields close() has to patch are known
  // constants rather than something to search for.
  bool write_header() {
    if (!write_fourcc("RIFF") || !write_u32(0) || !write_fourcc("AVI ")) return false;

    // 'hdrl' + avih chunk + strl list = 4 + 64 + 124.
    if (!write_fourcc("LIST") || !write_u32(192) || !write_fourcc("hdrl"))
      return false;

    // --- avih, the main header ---
    if (!write_fourcc("avih") || !write_u32(56)) return false;
    if (!write_u32(static_cast<std::uint32_t>(1000000 / fps_))) return false;  // us/frame
    if (!write_u32(0)) return false;          // max bytes/sec, 0 = unstated
    if (!write_u32(0)) return false;          // padding granularity
    if (!write_u32(0x00000010u)) return false;  // AVIF_HASINDEX
    if (!write_u32(0)) return false;          // total frames  <- patched
    if (!write_u32(0)) return false;          // initial frames
    if (!write_u32(1)) return false;          // streams
    if (!write_u32(0)) return false;          // suggested buffer size
    if (!write_u32(0)) return false;          // width   <- patched
    if (!write_u32(0)) return false;          // height  <- patched
    for (int i = 0; i < 4; ++i) if (!write_u32(0)) return false;  // reserved

    // --- strl: one video stream ---
    if (!write_fourcc("LIST") || !write_u32(4 + 8 + 56 + 8 + 40) ||
        !write_fourcc("strl")) return false;

    if (!write_fourcc("strh") || !write_u32(56)) return false;
    if (!write_fourcc("vids") || !write_fourcc("MJPG")) return false;
    if (!write_u32(0)) return false;          // flags
    if (!write_u16(0) || !write_u16(0)) return false;  // priority, language
    if (!write_u32(0)) return false;          // initial frames
    if (!write_u32(1)) return false;          // scale
    if (!write_u32(static_cast<std::uint32_t>(fps_))) return false;  // rate
    if (!write_u32(0)) return false;          // start
    if (!write_u32(0)) return false;          // length (frames)  <- patched
    if (!write_u32(0)) return false;          // suggested buffer size
    if (!write_u32(0xFFFFFFFFu)) return false;  // quality: unspecified
    if (!write_u32(0)) return false;          // sample size
    if (!write_u16(0) || !write_u16(0) || !write_u16(0) || !write_u16(0))
      return false;                           // rcFrame  <- patched

    if (!write_fourcc("strf") || !write_u32(40)) return false;
    if (!write_u32(40)) return false;         // biSize
    if (!write_u32(0)) return false;          // biWidth   <- patched
    if (!write_u32(0)) return false;          // biHeight  <- patched
    if (!write_u16(1) || !write_u16(24)) return false;  // planes, bit count
    if (!write_fourcc("MJPG")) return false;  // biCompression
    if (!write_u32(0)) return false;          // biSizeImage  <- patched
    for (int i = 0; i < 4; ++i) if (!write_u32(0)) return false;  // ppm, clr

    // --- movi: the frames themselves follow ---
    if (!write_fourcc("LIST") || !write_u32(0) || !write_fourcc("movi"))
      return false;                           // size <- patched

    // The layout above is fixed, so this is a constant. Asserting it here means
    // a mistake in the header shows up as a failed open on the first run rather
    // than as a file that plays as noise.
    movi_data_start_ = std::ftell(f_);
    return movi_data_start_ == kMoviDataStart;
  }

  // idx1: one 16 byte entry per frame. Offsets are relative to the 'movi'
  // FourCC, which is the convention every player accepts.
  bool write_index() {
    const long movi_end = std::ftell(f_);
    if (movi_end < 0) return false;
    movi_end_ = movi_end;

    if (!write_fourcc("idx1") ||
        !write_u32(static_cast<std::uint32_t>(index_.size() * 16))) return false;
    for (const entry& e : index_) {
      if (!write_fourcc("00dc")) return false;
      if (!write_u32(0x00000010u)) return false;  // AVIIF_KEYFRAME: every frame
      if (!write_u32(e.offset - static_cast<std::uint32_t>(movi_data_start_) + 4))
        return false;
      if (!write_u32(e.bytes)) return false;
    }
    file_end_ = std::ftell(f_);
    return file_end_ > 0;
  }

  // Absolute byte offsets of every field close() has to go back and fill in.
  //
  // These are derived by hand from write_header() and there is no way to check
  // them at compile time, so they are spelled out with the running total next
  // to each one. kMoviDataStart is checked at runtime, which catches any error
  // in the bytes before it -- that is every field in this list.
  //
  //    0 RIFF    4 size   8 'AVI '
  //   12 LIST   16 size  20 'hdrl'
  //   24 avih   28 size  32 avih data (56 bytes, 32..87)
  //   88 LIST   92 size  96 'strl'
  //  100 strh  104 size 108 strh data (56 bytes, 108..163)
  //  164 strf  168 size 172 strf data (40 bytes, 172..211)
  //  212 LIST  216 size 220 'movi'  224 first frame
  static constexpr long kRiffSize        = 4;
  static constexpr long kAvihMicroPerFrm = 32 + 0;
  static constexpr long kAvihTotalFrames = 32 + 16;
  static constexpr long kAvihBufferSize  = 32 + 28;
  static constexpr long kAvihWidth       = 32 + 32;
  static constexpr long kAvihHeight      = 32 + 36;
  static constexpr long kStrhRate        = 108 + 24;
  static constexpr long kStrhLength      = 108 + 32;
  static constexpr long kStrhBufferSize  = 108 + 36;
  static constexpr long kStrhRcRight     = 108 + 52;
  static constexpr long kStrfWidth       = 172 + 4;
  static constexpr long kStrfHeight      = 172 + 8;
  static constexpr long kStrfSizeImage   = 172 + 20;
  static constexpr long kMoviSize        = 216;
  static constexpr long kMoviDataStart   = 224;

  bool patch_header() {
    const std::uint32_t n = static_cast<std::uint32_t>(index_.size());
    std::uint32_t biggest = 0;
    for (const entry& e : index_) if (e.bytes > biggest) biggest = e.bytes;

    // rcFrame is four int16 -- left, top, right, bottom. left and top are
    // already zero, so one u32 sets the other two.
    const std::uint32_t rc_right_bottom =
        (static_cast<std::uint32_t>(h_) << 16) | static_cast<std::uint32_t>(w_);

    return patch_u32_at(kRiffSize, static_cast<std::uint32_t>(file_end_ - 8)) &&
           // Both places the rate is stated. set_fps() may have changed it
           // since the header went down, so neither can be left as written.
           patch_u32_at(kAvihMicroPerFrm,
                        static_cast<std::uint32_t>(1000000 / fps_)) &&
           patch_u32_at(kStrhRate, static_cast<std::uint32_t>(fps_)) &&
           patch_u32_at(kAvihTotalFrames, n) &&
           patch_u32_at(kAvihBufferSize, biggest) &&
           patch_u32_at(kAvihWidth, w_) &&
           patch_u32_at(kAvihHeight, h_) &&
           patch_u32_at(kStrhLength, n) &&
           patch_u32_at(kStrhBufferSize, biggest) &&
           patch_u32_at(kStrhRcRight, rc_right_bottom) &&
           patch_u32_at(kStrfWidth, w_) &&
           patch_u32_at(kStrfHeight, h_) &&
           patch_u32_at(kStrfSizeImage, biggest) &&
           patch_u32_at(kMoviSize,
                        static_cast<std::uint32_t>(movi_end_ - kMoviDataStart + 4));
  }

  std::FILE* f_ = nullptr;
  std::string path_;
  std::vector<entry> index_;
  long movi_data_start_ = 0;
  long movi_end_ = 0;
  long file_end_ = 0;
  std::uint16_t w_ = 0, h_ = 0;
  std::uint64_t rejected_ = 0;
  int fps_ = 4;
};

}  // namespace ls::avi
