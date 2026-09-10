// core/layout_hash.hpp : FNV-1a over a struct's field layout.
//
// Portable across compilers: field identities come from core/type_tag.hpp,
// which describes what a type IS rather than what a given compiler calls it.
//
// Two processes that disagree about a message's layout will silently misread
// each other's bytes. The bus refuses a connection unless both sides report the
// same 64-bit layout hash, which covers every field's name, type, offset and
// size plus the type's own size and alignment. Reordering two same-sized fields
// changes the hash; renaming a field changes the hash; adding padding changes
// the hash.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace ls {

struct field_desc {
  std::string_view name;
  std::string_view type;  // compiler's spelling, for the human-readable dump
  std::uint64_t type_tag;  // portable identity, for the hash -- core/type_tag.hpp
  std::size_t offset;
  std::size_t size;
};

namespace detail {

inline constexpr std::uint64_t fnv_offset_basis = 14695981039346656037ull;
inline constexpr std::uint64_t fnv_prime = 1099511628211ull;

constexpr std::uint64_t fnv1a(std::string_view s, std::uint64_t h) noexcept {
  for (const char c : s) {
    h ^= static_cast<std::uint8_t>(c);
    h *= fnv_prime;
  }
  return h;
}

constexpr std::uint64_t fnv1a(std::uint64_t v, std::uint64_t h) noexcept {
  for (int i = 0; i < 8; ++i) {
    h ^= static_cast<std::uint8_t>((v >> (i * 8)) & 0xffu);
    h *= fnv_prime;
  }
  return h;
}

constexpr std::uint64_t hash_field(const field_desc& f, std::uint64_t h) noexcept {
  h = fnv1a(f.name, h);
  // f.type_tag, NOT f.type: the compiler's spelling of a type differs between
  // toolchains ("unsigned __int64" vs "long unsigned int") and would make two
  // identical layouts hash differently. See core/type_tag.hpp.
  h = fnv1a(f.type_tag, h);
  h = fnv1a(static_cast<std::uint64_t>(f.offset), h);
  h = fnv1a(static_cast<std::uint64_t>(f.size), h);
  return h;
}

template <std::size_t N>
constexpr std::uint64_t layout_hash_of(std::string_view type,
                                       std::size_t size,
                                       std::size_t align,
                                       const field_desc (&fields)[N]) noexcept {
  std::uint64_t h = fnv1a(type, fnv_offset_basis);
  h = fnv1a(static_cast<std::uint64_t>(size), h);
  h = fnv1a(static_cast<std::uint64_t>(align), h);
  h = fnv1a(static_cast<std::uint64_t>(N), h);
  for (std::size_t i = 0; i < N; ++i) h = hash_field(fields[i], h);
  return h;
}

constexpr std::size_t round_up(std::size_t v, std::size_t a) noexcept {
  return ((v + a - 1) / a) * a;
}

// Verifies the declared field list actually accounts for the whole struct.
// Catches the most common LOCKSTEP_MESSAGE mistake: forgetting a member, which
// would leave bytes outside the hash and outside every relocation guarantee.
template <std::size_t N>
constexpr bool fields_cover_type(std::size_t size,
                                 std::size_t align,
                                 const field_desc (&fields)[N]) noexcept {
  std::size_t cursor = 0;
  for (std::size_t i = 0; i < N; ++i) {
    if (fields[i].offset < cursor) return false;  // out of declaration order
    cursor = fields[i].offset + fields[i].size;
    if (cursor > size) return false;
  }
  return round_up(cursor, align) == size;
}

// Padding detection lives in core/padding.hpp: it has to recurse into nested
// fields, and a flat sum of top-level field sizes cannot see that far.

}  // namespace detail
}  // namespace ls
