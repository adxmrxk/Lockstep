// The layout hash is the handshake that stops two processes built from
// different headers from silently misreading each other's bytes. These tests
// pin down what it must be sensitive to.
#include <cstdint>
#include <cstdio>

#include "lockstep/containers/inline_string.hpp"
#include "lockstep/containers/shm_span.hpp"
#include "lockstep/core/message.hpp"
#include "support/check.hpp"

struct HashA {
  std::uint32_t alpha;
  std::uint32_t beta;
};
LOCKSTEP_MESSAGE(HashA, alpha, beta);

// Identical layout, different type name.
struct HashB {
  std::uint32_t alpha;
  std::uint32_t beta;
};
LOCKSTEP_MESSAGE(HashB, alpha, beta);

// Same type name shape, fields swapped: same size, same alignment, different
// meaning. This is the case a naive size-and-count check would miss.
struct HashSwapped {
  std::uint32_t beta;
  std::uint32_t alpha;
};
LOCKSTEP_MESSAGE(HashSwapped, beta, alpha);

// Same names and offsets, one field's type widened.
struct HashWidened {
  std::uint32_t alpha;
  std::uint64_t beta;
};
LOCKSTEP_MESSAGE(HashWidened, alpha, beta);

// Same layout, a field renamed.
struct HashRenamed {
  std::uint32_t alpha;
  std::uint32_t gamma;
};
LOCKSTEP_MESSAGE(HashRenamed, alpha, gamma);

namespace {

template <class T>
constexpr std::uint64_t hash_of = ls::message_traits<T>::layout_hash;

}  // namespace

// GOLDEN VALUES. These are the whole point of the portable type tags: the same
// struct must hash to the same 64 bits on every compiler, or the connect-time
// handshake reports a layout mismatch between two nodes whose layouts are in
// fact identical. Both numbers below were produced by MSVC 19.29 AND by
// GCC 13.3 and compared by hand.
//
// If one of these fails, decide which it is before touching it:
//   * a deliberate format change  -> update the constant, and expect every
//     already-deployed node to refuse the new one, which is the hash doing its
//     job;
//   * an accidental change        -> you have just broken wire compatibility.
// Do not "fix" it by pasting in whatever the build printed.
struct GoldenImu {
  std::uint64_t stamp_ns;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
};
LOCKSTEP_MESSAGE(GoldenImu, stamp_ns, ax, ay, az, gx, gy, gz);

void the_hash_is_the_same_on_every_compiler() {
  // Scalars must not pick up the compiler's spelling of uint64_t.
  static_assert(ls::type_tag_v<std::uint64_t> == ls::type_tag_v<unsigned long long> ||
                    sizeof(unsigned long long) != 8,
                "two spellings of the same 64-bit unsigned type must tag alike");
  static_assert(ls::type_tag_v<std::int32_t> != ls::type_tag_v<std::uint32_t>,
                "signedness must still be part of a type's identity");
  static_assert(ls::type_tag_v<float> != ls::type_tag_v<std::uint32_t>,
                "a float and a uint32 are the same width and must not tag alike");

  std::printf("  [layout_hash] GoldenImu = 0x%016llx\n",
              (unsigned long long)ls::message_traits<GoldenImu>::layout_hash);

  // Produced by MSVC 19.29.30157 and by GCC 13.3.0 and compared by hand.
  // Before the portable type tags these two disagreed, because MSVC spells
  // std::uint64_t "unsigned __int64" and GCC spells it "long unsigned int",
  // and the spelling was going into the hash.
  LS_CHECK_EQ(ls::message_traits<GoldenImu>::layout_hash, 0xef6aa169e05c49c6ull);
}

int main() {
  the_hash_is_the_same_on_every_compiler();
  static_assert(hash_of<HashA> != 0);

  // Deterministic: the same type hashes the same every time it is asked.
  static_assert(hash_of<HashA> == ls::message_traits<HashA>::layout_hash);

  static_assert(hash_of<HashA> != hash_of<HashB>, "type name must be covered");
  static_assert(hash_of<HashA> != hash_of<HashSwapped>, "field order must be covered");
  static_assert(hash_of<HashA> != hash_of<HashWidened>, "field type must be covered");
  static_assert(hash_of<HashA> != hash_of<HashRenamed>, "field name must be covered");

  LS_CHECK(hash_of<HashA> != hash_of<HashSwapped>);
  LS_CHECK_EQ(ls::message_traits<HashSwapped>::field_count, 2u);
  return ls::test::summary("layout_hash");
}
