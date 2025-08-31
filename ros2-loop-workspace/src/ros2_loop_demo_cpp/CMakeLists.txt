cmake_minimum_required(VERSION 3.8)
project(ros2_loop_demo_cpp)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# Node A executable
add_executable(node_a src/node_a.cpp)
ament_target_dependencies(node_a
  rclcpp
  std_msgs
  rosbag2_cpp
  nlohmann_json
)

# Node B executable
add_executable(node_b src/node_b.cpp)
ament_target_dependencies(node_b
  rclcpp
  std_msgs
  nlohmann_json
)

# Install both executables
install(TARGETS
  node_a
  node_b
  DESTINATION lib/${PROJECT_NAME}
)

# Optional testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following lines skip linter checks for license/copyright
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
