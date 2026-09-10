// core/type_tag.hpp : a compiler-independent identity for a field's type.
//
// The layout hash is the handshake two peers trade at connect time, and it used
// to be built from `__FUNCSIG__` / `__PRETTY_FUNCTION__` scrapings. Those are
// compiler-specific spellings:
//
//     MSVC : unsigned __int64        struct ls::shm_span<unsigned char>
//     GCC  : long unsigned int       shm_span<unsigned char>
//
// So two nodes with byte-for-byte identical layouts, built with different
// compilers, computed different hashes and refused to talk to each other. The
// handshake was reporting a layout mismatch on types that matched perfectly.
// For a bus whose entire job is to connect processes, that is a real bug, and
// it is invisible until the day somebody builds one node with a different
// toolchain.
//
// The fix is to stop hashing what the type is CALLED and start hashing what it
// IS. A field's identity for wire-compatibility purposes is its width, its
// signedness, whether it is floating point, and -- for the aggregates this bus
// supports -- its shape. None of that varies by compiler.
//
// The scraped name is still carried in field_desc for the human-readable dump,
// where being able to read "unsigned __int64" is the point. It just no longer
// feeds the hash.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include "lockstep/core/type_name.hpp"

namespace ls {

namespace detail {

inline constexpr std::uint64_t tag_basis = 14695981039346656037ull;
inline constexpr std::uint64_t tag_prime = 1099511628211ull;

constexpr std::uint64_t tag_mix(std::string_view s, std::uint64_t h) noexcept {
  for (const char c : s) {
    h ^= static_cast<std::uint8_t>(c);
    h *= tag_prime;
  }
  return h;
}

constexpr std::uint64_t tag_mix(std::uint64_t v, std::uint64_t h) noexcept {
  for (int i = 0; i < 8; ++i) {
    h ^= static_cast<std::uint8_t>((v >> (i * 8)) & 0xffu);
    h *= tag_prime;
  }
  return h;
}

// Canonical spelling for a scalar, derived from its properties rather than from
// what the compiler happens to call it.
template <class T>
constexpr std::string_view scalar_name() noexcept {
  if constexpr (std::is_same_v<T, bool>) {
    return "b8";
  } else if constexpr (std::is_floating_point_v<T>) {
    return sizeof(T) == 4 ? "f32" : (sizeof(T) == 8 ? "f64" : "fXX");
  } else if constexpr (std::is_signed_v<T>) {
    return sizeof(T) == 1   ? "i8"
           : sizeof(T) == 2 ? "i16"
           : sizeof(T) == 4 ? "i32"
           : sizeof(T) == 8 ? "i64"
                            : "iXX";
  } else {
    return sizeof(T) == 1   ? "u8"
           : sizeof(T) == 2 ? "u16"
           : sizeof(T) == 4 ? "u32"
           : sizeof(T) == 8 ? "u64"
                            : "uXX";
  }
}

}  // namespace detail

// Primary template. Scalars and enums resolve structurally; everything else
// must specialize, and the containers and LOCKSTEP_MESSAGE do.
//
// The fallback for an unrecognised class type is the scraped compiler name,
// which is NOT portable across compilers -- but such a type cannot be a message
// field anyway, because is_relocatable rejects it. It exists so the trait is
// total rather than a hard error.
template <class T>
struct type_tag_of {
  static constexpr std::uint64_t value = [] {
    if constexpr (std::is_enum_v<T>) {
      return detail::tag_mix(
          detail::scalar_name<std::underlying_type_t<T>>(),
          detail::tag_mix(std::string_view("enum"), detail::tag_basis));
    } else if constexpr (std::is_arithmetic_v<T>) {
      return detail::tag_mix(detail::scalar_name<T>(), detail::tag_basis);
    } else {
      return detail::tag_mix(type_name<T>(), detail::tag_basis);
    }
  }();
};

template <class T>
inline constexpr std::uint64_t type_tag_v = type_tag_of<T>::value;

// C arrays: element identity plus extent.
template <class T, std::size_t N>
struct type_tag_of<T[N]> {
  static constexpr std::uint64_t value = detail::tag_mix(
      static_cast<std::uint64_t>(N),
      detail::tag_mix(type_tag_v<T>,
                      detail::tag_mix(std::string_view("arr"), detail::tag_basis)));
};

}  // namespace ls
