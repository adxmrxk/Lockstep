// The mapping layer: create, attach, address translation, teardown.
#include <cstdint>
#include <cstring>
#include <string>
#include <system_error>
#include <unistd.h>

#include "lockstep/shm/segment.hpp"
#include "support/check.hpp"

namespace {

std::string unique_name(const char* tag) {
  return std::string("lockstep-test-") + tag + "-" + std::to_string(::getpid());
}

void create_map_and_unlink() {
  const std::string name = unique_name("basic");
  ls::segment::unlink(name);  // in case a previous run died holding it

  {
    ls::segment s = ls::segment::create(name, 64 * 1024);
    LS_CHECK(s.valid());
    LS_CHECK(s.owner());
    LS_CHECK_EQ(s.size(), 64u * 1024u);
    LS_CHECK(s.base() != nullptr);
  }

  // The owner unlinked on destruction, so opening it again must fail.
  bool threw = false;
  try {
    ls::segment gone = ls::segment::open(name);
  } catch (const std::system_error&) {
    threw = true;
  }
  LS_CHECK(threw);
}

void two_mappings_of_one_object_share_bytes() {
  const std::string name = unique_name("shared");
  ls::segment::unlink(name);

  ls::segment owner = ls::segment::create(name, 64 * 1024);
  ls::segment viewer = ls::segment::open(name);

  LS_CHECK(viewer.valid());
  LS_CHECK(!viewer.owner());
  LS_CHECK_EQ(viewer.size(), owner.size());

  // A write through one mapping must be visible through the other. Within one
  // process the two mappings usually land at different addresses, which is a
  // small-scale version of what two processes see.
  auto* w = owner.at<std::uint64_t>(4096);
  *w = 0xfeedfacecafebeefull;
  LS_CHECK_EQ(*viewer.at<std::uint64_t>(4096), 0xfeedfacecafebeefull);

  *viewer.at<std::uint64_t>(4096) = 7;
  LS_CHECK_EQ(*w, 7u);
}

void offsets_round_trip() {
  const std::string name = unique_name("offsets");
  ls::segment::unlink(name);
  ls::segment s = ls::segment::create(name, 64 * 1024);

  void* p = s.at<void>(1234);
  LS_CHECK_EQ(s.offset_of(p), 1234u);
  LS_CHECK(s.contains(p));
  LS_CHECK(s.contains(s.at<void>(0), s.size()));
  LS_CHECK(!s.contains(s.at<void>(s.size() + 8)));
}

void create_refuses_to_clobber_an_existing_name() {
  const std::string name = unique_name("exclusive");
  ls::segment::unlink(name);

  ls::segment first = ls::segment::create(name, 4096);
  bool threw = false;
  try {
    ls::segment second = ls::segment::create(name, 4096);
  } catch (const std::system_error&) {
    threw = true;
  }
  LS_CHECK(threw);
}

void moving_a_segment_transfers_ownership() {
  const std::string name = unique_name("move");
  ls::segment::unlink(name);

  ls::segment a = ls::segment::create(name, 4096);
  void* base = a.base();

  ls::segment b = std::move(a);
  LS_CHECK(b.valid());
  LS_CHECK_EQ(b.base(), base);
  LS_CHECK(!a.valid());  // moved-from is empty, not a double-unlink waiting to happen

  // b still owns the name, so opening it must still work.
  ls::segment viewer = ls::segment::open(name);
  LS_CHECK(viewer.valid());
}

}  // namespace

int main() {
  create_map_and_unlink();
  two_mappings_of_one_object_share_bytes();
  offsets_round_trip();
  create_refuses_to_clobber_an_existing_name();
  moving_a_segment_transfers_ownership();
  return ls::test::summary("segment");
}
