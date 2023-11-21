cmake_minimum_required(VERSION 3.8)
project(sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Find the library
find_library(SERIAL_MEGAPI NAMES serial_megapi PATHS /usr/local/lib)

# Sensor Range Node
add_executable(range src/range.cpp)
target_link_libraries(range ${SERIAL_MEGAPI})
ament_target_dependencies(range rclcpp std_msgs sensor_msgs)

# Driver Node
add_executable(driver src/driver.cpp)
target_link_libraries(driver ${SERIAL_MEGAPI})
ament_target_dependencies(driver rclcpp geometry_msgs)

install(TARGETS range driver DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

