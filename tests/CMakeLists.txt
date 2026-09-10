# Positive tests: plain executables that return non-zero on failure.
set(LOCKSTEP_TESTS
  test_offset_ptr
  test_relocation
  test_containers
  test_message
  test_layout_hash
  test_provenance)

foreach(t IN LISTS LOCKSTEP_TESTS)
  add_executable(${t} ${t}.cpp)
  target_link_libraries(${t} PRIVATE lockstep::lockstep)
  target_include_directories(${t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME ${t} COMMAND ${t})
endforeach()

# Negative compile tests: the type system's job is to REJECT these, so the
# assertion is that the build fails. Each is excluded from the default build and
# driven through CTest with WILL_FAIL.
set(LOCKSTEP_COMPILE_FAIL_TESTS
  raw_pointer_field
  incomplete_field_list
  std_string_field)

foreach(t IN LISTS LOCKSTEP_COMPILE_FAIL_TESTS)
  add_executable(compile_fail_${t} EXCLUDE_FROM_ALL negative/${t}.cpp)
  target_link_libraries(compile_fail_${t} PRIVATE lockstep::lockstep)
  add_test(NAME compile_fail.${t}
           COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}
                                    --target compile_fail_${t}
                                    --config $<CONFIG>)
  set_tests_properties(compile_fail.${t} PROPERTIES WILL_FAIL TRUE)
endforeach()

# ---------------------------------------------------------------------------
# Phase 2: shared memory. POSIX only for now -- the Win32 CreateFileMapping
# backend is not written, so these targets are skipped on Windows rather than
# failing the build there.
# ---------------------------------------------------------------------------
if(UNIX)
  find_package(Threads REQUIRED)

  # shm_open lives in librt on glibc older than 2.34, and in libc after it.
  find_library(LOCKSTEP_RT_LIB rt)
  set(LOCKSTEP_SHM_LIBS lockstep::lockstep Threads::Threads)
  if(LOCKSTEP_RT_LIB)
    list(APPEND LOCKSTEP_SHM_LIBS ${LOCKSTEP_RT_LIB})
  endif()

  set(LOCKSTEP_SHM_TESTS
    test_segment
    test_arena
    test_registry
    test_ring
    test_pubsub
    test_executor)

  foreach(t IN LISTS LOCKSTEP_SHM_TESTS)
    add_executable(${t} ${t}.cpp)
    target_link_libraries(${t} PRIVATE ${LOCKSTEP_SHM_LIBS})
    target_include_directories(${t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    add_test(NAME ${t} COMMAND ${t})
  endforeach()

  # The acceptance test needs a genuinely separate binary to exec, so that the
  # second process shares no address space with the first.
  add_executable(two_process_child two_process_child.cpp)
  target_link_libraries(two_process_child PRIVATE ${LOCKSTEP_SHM_LIBS})
  target_include_directories(two_process_child PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

  # Phase 5: crash consistency. Needs its own victim binary to SIGKILL.
  add_executable(crash_publisher_child crash_publisher_child.cpp)
  target_link_libraries(crash_publisher_child PRIVATE ${LOCKSTEP_SHM_LIBS})
  target_include_directories(crash_publisher_child PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

  add_executable(test_crash_consistency test_crash_consistency.cpp)
  target_link_libraries(test_crash_consistency PRIVATE ${LOCKSTEP_SHM_LIBS})
  target_include_directories(test_crash_consistency PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_definitions(test_crash_consistency PRIVATE
    LOCKSTEP_CRASH_CHILD_BINARY="$<TARGET_FILE:crash_publisher_child>")
  add_dependencies(test_crash_consistency crash_publisher_child)
  add_test(NAME test_crash_consistency COMMAND test_crash_consistency)
  set_tests_properties(test_crash_consistency PROPERTIES TIMEOUT 600)

  add_executable(test_two_process test_two_process.cpp)
  target_link_libraries(test_two_process PRIVATE ${LOCKSTEP_SHM_LIBS})
  target_include_directories(test_two_process PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_definitions(test_two_process PRIVATE
    LOCKSTEP_CHILD_BINARY="$<TARGET_FILE:two_process_child>")
  add_dependencies(test_two_process two_process_child)
  add_test(NAME test_two_process COMMAND test_two_process)
endif()
