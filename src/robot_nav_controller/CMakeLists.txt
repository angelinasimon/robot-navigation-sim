cmake_minimum_required(VERSION 3.8)
project(robot_nav_controller LANGUAGES CXX)

add_library(robot_nav_controller src/controller.cpp src/utils.cpp src/pose.cpp src/command.cpp)
target_include_directories(robot_nav_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_nav_controller PUBLIC cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(robot_nav_controller PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
  foreach(test_name controller_unit closed_loop_drive)
    add_executable(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE robot_nav_controller)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()

  # Experiments print measurements for a human to read; they have no pass/fail,
  # so they are built but not registered with ctest.
  add_executable(wraparound_study experiments/wraparound_study.cpp)
  target_link_libraries(wraparound_study PRIVATE robot_nav_controller)
  target_include_directories(wraparound_study PRIVATE test)
endif()

install(TARGETS robot_nav_controller DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

# Preserve colcon package discovery when built in a ROS workspace.
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_export_include_directories(include)
  ament_export_libraries(robot_nav_controller)
  ament_package()
endif()
