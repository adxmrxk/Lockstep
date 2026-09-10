// core/type_name.hpp : compile-time type names, used as layout-hash input.
#pragma once

#include <string_view>

namespace ls {

// Extracts the spelled-out type name from the compiler's function signature
// macro. Used only to feed the layout hash, so exact spelling does not matter
// as long as it is stable for a given compiler and differs between types.
//
// GCC appends the other template parameters of the enclosing scope to
// __PRETTY_FUNCTION__, so the signature reads
//
//     ... ls::type_name() [with T = long unsigned int; std::string_view = ...]
//
// and stopping at the final ']' drags that tail into the name. Stop at the
// first ';' after the parameter instead, and fall back to ']' on Clang, which
// emits no tail.
template <class T>
constexpr std::string_view type_name() noexcept {
#if defined(_MSC_VER)
  constexpr std::string_view sig{__FUNCSIG__};
  constexpr std::string_view open{"type_name<"};
  const auto start = sig.find(open) + open.size();
  const auto end = sig.rfind(">(");
#else
  constexpr std::string_view sig{__PRETTY_FUNCTION__};
  constexpr std::string_view open{"T = "};
  const auto start = sig.find(open) + open.size();
  const auto semi = sig.find(';', start);
  const auto end = (semi == std::string_view::npos) ? sig.rfind(']') : semi;
#endif
  return (start >= end || end == std::string_view::npos)
             ? sig
             : sig.substr(start, end - start);
}

}  // namespace ls
