# The ROS 2 bridge.
#
# Entirely optional, like the Cyclone benchmark. If rclcpp is not found this
# directory adds nothing and the rest of the project builds exactly as before.
# The core stays dependency-free on purpose -- ROS is what this bridges TO, not
# something Lockstep is built on.
#
# To build it, source a ROS 2 install first:
#   source /opt/ros/jazzy/setup.bash
#   cmake -S . -B build
find_package(rclcpp QUIET)
find_package(sensor_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(builtin_interfaces QUIET)

if(NOT (rclcpp_FOUND AND sensor_msgs_FOUND AND geometry_msgs_FOUND))
  message(STATUS "ROS 2 (rclcpp) not found -- skipping the ROS bridge. "
                 "Source a ROS 2 setup.bash and re-configure to enable it.")
  return()
endif()

message(STATUS "ROS 2 found -- building the ROS bridge")

set(LOCKSTEP_ROS_LIBS
  lockstep::lockstep
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${builtin_interfaces_TARGETS})

find_package(Threads REQUIRED)
list(APPEND LOCKSTEP_ROS_LIBS Threads::Threads)
find_library(LOCKSTEP_RT_LIB rt)
if(LOCKSTEP_RT_LIB)
  list(APPEND LOCKSTEP_ROS_LIBS ${LOCKSTEP_RT_LIB})
endif()

add_executable(ros_bridge ros_bridge.cpp)
target_link_libraries(ros_bridge PRIVATE ${LOCKSTEP_ROS_LIBS})
target_include_directories(ros_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# The conversions are pure functions, so they are tested with no ROS graph
# running -- no daemon, no discovery, no second process.
add_executable(test_ros_bridge test_ros_bridge.cpp)
target_link_libraries(test_ros_bridge PRIVATE ${LOCKSTEP_ROS_LIBS})
target_include_directories(test_ros_bridge PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/tests)
add_test(NAME test_ros_bridge COMMAND test_ros_bridge)
