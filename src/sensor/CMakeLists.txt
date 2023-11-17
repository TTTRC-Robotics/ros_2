cmake_minimum_required(VERSION 3.8)
project(sensor)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)  # Add sensor_msgs dependency
find_package(std_msgs REQUIRED)

find_library(SERIAL_MEGAPI serial_megapi PATHS /usr/local/lib)

if(NOT SERIAL_MEGAPI)
    message(FATAL_ERROR "Library serial_megapi not found")
endif()

add_executable(range src/range.cpp)
target_link_libraries(range ${SERIAL_MEGAPI})
ament_target_dependencies(range rclcpp sensor_msgs std_msgs)  # Add sensor_msgs dependency

install(TARGETS
  range
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license are added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
