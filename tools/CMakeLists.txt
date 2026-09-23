# Operator tooling. Read-only observers of a running bus.
if(NOT (UNIX OR WIN32))
  return()
endif()

find_package(Threads REQUIRED)
set(LOCKSTEP_TOOL_LIBS lockstep::lockstep Threads::Threads)
if(UNIX)
  find_library(LOCKSTEP_RT_LIB rt)
  if(LOCKSTEP_RT_LIB)
    list(APPEND LOCKSTEP_TOOL_LIBS ${LOCKSTEP_RT_LIB})
  endif()
endif()

add_executable(lockstep_top lockstep_top.cpp)
target_link_libraries(lockstep_top PRIVATE ${LOCKSTEP_TOOL_LIBS})
