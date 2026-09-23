cmake_minimum_required(VERSION 3.20)
project(lockstep VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# Phase 1 is header-only. The shared-memory transport in phase 2 will add a
# compiled component behind this same target.
add_library(lockstep INTERFACE)
add_library(lockstep::lockstep ALIAS lockstep)

target_include_directories(lockstep INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(lockstep INTERFACE cxx_std_20)

if(MSVC)
  # /Zc:preprocessor -- LOCKSTEP_MESSAGE relies on conformant __VA_ARGS__
  #                     expansion, which the legacy MSVC preprocessor breaks.
  # /permissive-     -- two-phase lookup, required for the trait templates.
  # /wd5105 -- the Windows SDK's own winbase.h produces "macro expansion
  #            producing 'defined' has undefined behavior" when compiled with
  #            /Zc:preprocessor, which LOCKSTEP_MESSAGE requires. It is the
  #            SDK's defect, not this project's, and there is no way to fix it
  #            from here short of giving up conformant __VA_ARGS__ expansion.
  #            Scoped to this warning only, so our own code stays /W4-clean.
  target_compile_options(lockstep INTERFACE /Zc:preprocessor /permissive- /W4 /wd5105)
else()
  target_compile_options(lockstep INTERFACE -Wall -Wextra -Wpedantic)
endif()

enable_testing()
add_subdirectory(tests)
add_subdirectory(examples)
