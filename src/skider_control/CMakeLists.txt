cmake_minimum_required(VERSION 3.8)
project(skider_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(skider_interface REQUIRED)
find_package(skider_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# NODE1 : gimbal_demo_node
add_executable(gimbal_demo_node
  src/gimbal_demo_node.cpp
)
target_include_directories(gimbal_demo_node PUBLIC
  include/skider_control
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(gimbal_demo_node
  rclcpp
  rclcpp_components
  skider_interface
  skider_utils
  geometry_msgs
  sensor_msgs
  Eigen3
)

# NODE2 : chassis_demo_node
add_executable(chassis_demo_node
  src/chassis_demo_node.cpp
)
target_include_directories(chassis_demo_node PUBLIC
  include/skider_control
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(chassis_demo_node
  rclcpp
  rclcpp_components
  skider_interface
  skider_utils
  sensor_msgs
  Eigen3
)


# Install executable nodes
install(TARGETS gimbal_demo_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS chassis_demo_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
