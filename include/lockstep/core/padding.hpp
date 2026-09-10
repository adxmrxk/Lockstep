// core/padding.hpp : how many bytes of a type no field accounts for.
//
// Determinism, not tidiness. A replay hash is taken over a whole message slot,
// so a struct carrying indeterminate padding bytes hashes differently run to
// run unless the publisher zero-fills it first. message_traits<T>::has_padding
// is what tells the publisher to do that, which means it has to see padding at
// every depth -- including padding buried inside a nested field, which a
// top-level sum-of-field-sizes check cannot see:
//
//   struct Inner { std::uint8_t a; std::uint64_t b; };   // 16 bytes, 7 padding
//   struct Outer { Inner inner; std::uint64_t c; };      // 24 bytes
//
// Outer's field sizes sum to exactly sizeof(Outer), so a flat check calls it
// padding-free while 7 indeterminate bytes sit inside it. Recursing fixes that.
#pragma once

#include <array>
#include <cstddef>
#include <type_traits>

namespace ls {

// Scalars and enums have no padding. Aggregates specialize this: the containers
// in their own headers, and every LOCKSTEP_MESSAGE type through the macro.
//
// A class type that specializes nothing reports zero, which is safe here only
// because LOCKSTEP_MESSAGE rejects any member that is neither a scalar, an
// array, a declared message, nor one of the containers -- see is_relocatable.
template <class T>
struct padding_bytes : std::integral_constant<std::size_t, 0> {};

template <class T, std::size_t N>
struct padding_bytes<T[N]>
    : std::integral_constant<std::size_t, N * padding_bytes<T>::value> {};

template <class T, std::size_t N>
struct padding_bytes<std::array<T, N>>
    : std::integral_constant<std::size_t,
                             sizeof(std::array<T, N>) - N * (sizeof(T) - padding_bytes<T>::value)> {};

template <class T>
inline constexpr std::size_t padding_bytes_v = padding_bytes<T>::value;

// The bytes of T that a field actually accounts for.
template <class T>
inline constexpr std::size_t useful_bytes_v = sizeof(T) - padding_bytes_v<T>;

// Folded by LOCKSTEP_MESSAGE over a declared field list.
template <class... Ts>
inline constexpr std::size_t total_useful_bytes_v = (useful_bytes_v<Ts> + ... + 0u);

}  // namespace ls
