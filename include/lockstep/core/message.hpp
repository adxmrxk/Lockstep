// core/message.hpp : LOCKSTEP_MESSAGE -- opt a struct into the bus.
//
//   struct ImuSample {
//     std::uint64_t stamp_ns;
//     float ax, ay, az;
//   };
//   LOCKSTEP_MESSAGE(ImuSample, stamp_ns, ax, ay, az);
//
// The macro must appear at global scope, after the struct definition, naming
// every member in declaration order. In exchange it:
//
//   * static_asserts that every member is Relocatable, so a raw pointer,
//     std::string or std::vector member is a compile error rather than a
//     subscriber reading garbage;
//   * static_asserts the field list actually covers sizeof(T), so a forgotten
//     member cannot slip past the layout hash;
//   * computes a 64-bit layout hash the bus exchanges at connect time;
//   * reports whether the struct has padding at any depth, which the publisher
//     must zero-fill or deterministic replay hashes will not reproduce. Padding
//     inside a nested field counts -- see core/padding.hpp.
//
// Limitation: Type must not contain a comma, so template specializations must
// be given an alias first.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <type_traits>

#include "lockstep/core/detail/foreach.inc"
#include "lockstep/core/layout_hash.hpp"
#include "lockstep/core/padding.hpp"
#include "lockstep/core/relocatable.hpp"
#include "lockstep/core/type_name.hpp"
#include "lockstep/core/type_tag.hpp"

namespace ls {

// Specialized by LOCKSTEP_MESSAGE. The unspecialized form is what the bus sees
// when someone tries to publish a type that was never declared.
template <class T>
struct message_traits {
  static constexpr bool declared = false;
};

template <class T>
concept Message = message_traits<T>::declared && ZeroCopyable<T>;

}  // namespace ls

#define LS_MSG_FIELD(Type, member)                                       \
  ::ls::field_desc {                                                     \
    #member, ::ls::type_name<decltype(Type::member)>(),                  \
        ::ls::type_tag_v<decltype(Type::member)>,                        \
        offsetof(Type, member), sizeof(Type::member)                     \
  }

#define LS_MSG_TYPE(Type, member) decltype(Type::member)

#define LOCKSTEP_MESSAGE(Type, ...)                                            \
  namespace ls {                                                               \
  template <>                                                                  \
  struct padding_bytes<Type>                                                   \
      : std::integral_constant<                                                \
            std::size_t,                                                       \
            sizeof(Type) - ::ls::total_useful_bytes_v<                         \
                               LS_MSG_FOR_EACH(LS_MSG_TYPE, Type,              \
                                               __VA_ARGS__)>> {};              \
  template <>                                                                  \
  struct message_traits<Type> {                                                \
    static constexpr bool declared = true;                                     \
    static constexpr std::string_view name = #Type;                            \
    static constexpr std::size_t field_count = LS_MSG_NARG(__VA_ARGS__);       \
    static constexpr std::size_t size = sizeof(Type);                          \
    static constexpr std::size_t align = alignof(Type);                        \
    static constexpr ::ls::field_desc fields[field_count] = {                  \
        LS_MSG_FOR_EACH(LS_MSG_FIELD, Type, __VA_ARGS__)};                     \
    static constexpr std::size_t padding = ::ls::padding_bytes_v<Type>;        \
    static constexpr bool has_padding = padding != 0;                          \
    static constexpr std::uint64_t layout_hash =                               \
        ::ls::detail::layout_hash_of(name, size, align, fields);               \
  };                                                                           \
  template <>                                                                  \
  struct type_tag_of<Type> {                                                   \
    /* A declared message's identity IS its layout hash, so a nested message   \
       field contributes portably to its parent's hash. */                     \
    static constexpr std::uint64_t value =                                     \
        ::ls::message_traits<Type>::layout_hash;                               \
  };                                                                           \
  template <>                                                                  \
  struct is_relocatable<Type>                                                  \
      : std::bool_constant<::ls::all_relocatable_v<                            \
            LS_MSG_FOR_EACH(LS_MSG_TYPE, Type, __VA_ARGS__)>> {};              \
  }                                                                            \
  static_assert(                                                               \
      ::ls::all_relocatable_v<LS_MSG_FOR_EACH(LS_MSG_TYPE, Type, __VA_ARGS__)>, \
      "LOCKSTEP_MESSAGE(" #Type                                                \
      "): a member is not relocatable. Raw pointers, references, std::string "  \
      "and std::vector cannot cross a shared-memory boundary -- they encode "   \
      "addresses that are meaningless in another process. Use ls::offset_ptr, " \
      "ls::inline_string<N>, ls::inline_vector<T, N> or ls::shm_span<T>.");     \
  static_assert(std::is_standard_layout_v<Type>,                               \
                "LOCKSTEP_MESSAGE(" #Type                                      \
                "): must be standard layout (no mixed access control, no "     \
                "virtuals, no base class with data members).");                \
  static_assert(std::is_trivially_copyable_v<Type>,                            \
                "LOCKSTEP_MESSAGE(" #Type                                      \
                "): must be trivially copyable -- the bus relocates it with "  \
                "memcpy and never runs a copy constructor.");                  \
  static_assert(std::is_trivially_destructible_v<Type>,                        \
                "LOCKSTEP_MESSAGE(" #Type                                      \
                "): must be trivially destructible -- shared memory outlives " \
                "the process that wrote it and no destructor will ever run.");  \
  static_assert(::ls::detail::fields_cover_type(sizeof(Type), alignof(Type),   \
                                                ::ls::message_traits<Type>::fields), \
                "LOCKSTEP_MESSAGE(" #Type                                      \
                "): the field list does not account for every byte of the "    \
                "struct. List every member, in declaration order.");           \
  static_assert(true, "")
