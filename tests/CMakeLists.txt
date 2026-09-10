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
