add_executable(example_message_layout 01_message_layout.cpp)
target_link_libraries(example_message_layout PRIVATE lockstep::lockstep)

# The five-node demo needs the shared-memory transport, so it is POSIX-only for
# the same reason the phase 2+ tests are.
if(UNIX)
  find_package(Threads REQUIRED)
  find_library(LOCKSTEP_RT_LIB rt)
  set(LOCKSTEP_DEMO_LIBS lockstep::lockstep Threads::Threads)
  if(LOCKSTEP_RT_LIB)
    list(APPEND LOCKSTEP_DEMO_LIBS ${LOCKSTEP_RT_LIB})
  endif()

  add_executable(demo_node demo_node.cpp)
  target_link_libraries(demo_node PRIVATE ${LOCKSTEP_DEMO_LIBS})
  target_include_directories(demo_node PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

  add_executable(five_node_demo 05_five_node_demo.cpp)
  target_link_libraries(five_node_demo PRIVATE ${LOCKSTEP_DEMO_LIBS})
  target_include_directories(five_node_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_definitions(five_node_demo PRIVATE
    LOCKSTEP_DEMO_NODE_BINARY="$<TARGET_FILE:demo_node>")
  add_dependencies(five_node_demo demo_node)

  add_executable(bench_latency ../benchmarks/bench_latency.cpp)
  target_link_libraries(bench_latency PRIVATE ${LOCKSTEP_DEMO_LIBS})
  target_include_directories(bench_latency PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
endif()
