# The Cyclone DDS baseline.
#
# Entirely optional: if CycloneDDS is not found this directory adds nothing and
# the rest of the project builds exactly as before. Cyclone is a comparison
# point, not a dependency -- the core stays dependency-free on purpose, since a
# message bus is the thing every other process on the robot links against.
#
# To build it, point CMake at an install:
#   cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/cdds
#
# It does not need root. scripts/build-cyclone.sh builds one into $HOME.

find_package(CycloneDDS QUIET)

if(NOT CycloneDDS_FOUND)
  message(STATUS "CycloneDDS not found -- skipping the DDS comparison benchmark. "
                 "Run scripts/build-cyclone.sh and re-configure with "
                 "-DCMAKE_PREFIX_PATH=$ENV{HOME}/cdds to enable it.")
  return()
endif()

message(STATUS "CycloneDDS found -- building the DDS comparison benchmark")

# The Cyclone API and the idlc-generated types are C. Enabled HERE rather than
# in the top-level project() so the core keeps requiring nothing but a C++
# compiler -- this is a comparison point, and it should not add a build
# requirement to people who never run it.
enable_language(C)

# idlc generates the C type from BenchMsg.idl at build time, so the generated
# code is never checked in and cannot drift from the IDL.
idlc_generate(TARGET bench_msg_types FILES BenchMsg.idl WARNINGS no-implicit-extensibility)

add_executable(bench_cyclone bench_cyclone.c)
target_link_libraries(bench_cyclone PRIVATE bench_msg_types CycloneDDS::ddsc)

# The comparison harness only makes sense when both halves exist.
set(LOCKSTEP_HAVE_CYCLONE TRUE PARENT_SCOPE)
