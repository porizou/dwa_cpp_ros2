cmake_minimum_required(VERSION 3.8)
project(dwa_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(dwa_ros2 SHARED
  src/obstacle_detector_component.cpp
  src/dwa_ros2.cpp
)

ament_target_dependencies(dwa_ros2 rclcpp  sensor_msgs nav_msgs rclcpp_components tf2 tf2_geometry_msgs)

rclcpp_components_register_nodes(dwa_ros2 "dwa_ros2::ObstacleDetector")
rclcpp_components_register_nodes(dwa_ros2 "dwa_ros2::DWA")

install(TARGETS ${PROJECT_NAME} DESTINATION lib )
# Call ament build system
ament_package()
