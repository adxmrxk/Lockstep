# The Elegoo robot bridge.
#
# Needs the shared-memory transport and sockets, both of which exist on either
# platform, so this builds everywhere the bus does.
if(NOT (UNIX OR WIN32))
  return()
endif()

find_package(Threads REQUIRED)

set(LOCKSTEP_ROBOT_LIBS lockstep::lockstep Threads::Threads)
if(UNIX)
  find_library(LOCKSTEP_RT_LIB rt)
  if(LOCKSTEP_RT_LIB)
    list(APPEND LOCKSTEP_ROBOT_LIBS ${LOCKSTEP_RT_LIB})
  endif()
else()
  # Winsock. The only library this project links that is not the C++ runtime.
  list(APPEND LOCKSTEP_ROBOT_LIBS ws2_32)
endif()

add_executable(elegoo_bridge bridge.cpp)
target_link_libraries(elegoo_bridge PRIVATE ${LOCKSTEP_ROBOT_LIBS})
target_include_directories(elegoo_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(avoider avoider.cpp)
target_link_libraries(avoider PRIVATE ${LOCKSTEP_ROBOT_LIBS})
target_include_directories(avoider PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# The robot tests run with no robot: the protocol encoders are pure functions,
# and the link test stands up a fake car on loopback. Everything except the
# actual Elegoo firmware is therefore covered.
add_executable(test_elegoo ${CMAKE_CURRENT_SOURCE_DIR}/test_elegoo.cpp)
target_link_libraries(test_elegoo PRIVATE ${LOCKSTEP_ROBOT_LIBS})
target_include_directories(test_elegoo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_elegoo COMMAND test_elegoo)
