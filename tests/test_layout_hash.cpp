// The layout hash is the handshake that stops two processes built from
// different headers from silently misreading each other's bytes. These tests
// pin down what it must be sensitive to.
#include <cstdint>

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

int main() {
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
