// core/relocatable.hpp : the type system that keeps raw pointers out of shared
// memory.
//
// A type is Relocatable when a byte-for-byte copy of its storage to a different
// address is still a valid object with all internal links intact. That is
// strictly stronger than std::is_trivially_copyable, because a raw pointer is
// trivially copyable but does not survive relocation.
//
// C++20 has no reflection, so we cannot walk a struct's members to prove the
// absence of raw pointers. Instead, class types are NOT relocatable by default;
// they must opt in through LOCKSTEP_MESSAGE, which checks every declared member
// against this trait. That inverts the default from "silently unsafe" to
// "refuses to compile".
#pragma once

#include <array>
#include <cstddef>
#include <type_traits>

#include "lockstep/core/offset_ptr.hpp"

namespace ls {

namespace detail {
// Scalars relocate trivially. Class types fall through to false and must opt in
// via LOCKSTEP_MESSAGE. Pointers are arithmetic-free and land on false here,
// but we also reject them explicitly below so the failure names itself.
template <class T>
struct relocatable_default
    : std::bool_constant<std::is_arithmetic_v<T> || std::is_enum_v<T>> {};
}  // namespace detail

template <class T>
struct is_relocatable : detail::relocatable_default<T> {};

template <class T>
struct is_relocatable<offset_ptr<T>> : std::true_type {};

template <class T, std::size_t N>
struct is_relocatable<T[N]> : is_relocatable<T> {};

template <class T, std::size_t N>
struct is_relocatable<std::array<T, N>> : is_relocatable<T> {};

// Explicit rejections. These are what shared-memory bug reports are usually
// made of, so name them rather than letting them fall through silently.
template <class T>
struct is_relocatable<T*> : std::false_type {};
template <class T>
struct is_relocatable<T&> : std::false_type {};
template <class T>
struct is_relocatable<T&&> : std::false_type {};

template <class T>
inline constexpr bool is_relocatable_v = is_relocatable<T>::value;

template <class T>
concept Relocatable = is_relocatable_v<T>;

// The full contract a type must satisfy to cross a shared-memory boundary.
//   standard_layout        -- so offsetof() in the message traits is defined
//   trivially_copyable     -- so the bus may memcpy the message as raw bytes
//   trivially_destructible -- shared memory outlives the writing process and
//                             nobody will ever run a destructor in it
template <class T>
concept ZeroCopyable =
    Relocatable<T> && std::is_standard_layout_v<T> &&
    std::is_trivially_copyable_v<T> && std::is_trivially_destructible_v<T>;

}  // namespace ls

namespace ls {

// Fold used by LOCKSTEP_MESSAGE to check a declared field list in one pass.
template <class... Ts>
inline constexpr bool all_relocatable_v = (is_relocatable_v<Ts> && ...);

}  // namespace ls
