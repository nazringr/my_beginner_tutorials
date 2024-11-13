cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# Check whether compiler is GCC or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard to C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable compile commands for linting
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Enable custom service generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetString.srv"
  DEPENDENCIES std_msgs
)

# ROS Interface dependencies for custom service
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Build talker - publisher
add_executable(talker src/publisher_node.cpp)
ament_target_dependencies(talker rclcpp std_msgs rosidl_default_runtime)
target_link_libraries(talker "${cpp_typesupport_target}")

# Build listener - subscriber
add_executable(listener src/subscriber_node.cpp)
ament_target_dependencies(listener rclcpp std_msgs rosidl_default_runtime)
target_link_libraries(listener "${cpp_typesupport_target}")

# Install binaries
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})


# Install the srv files for service
install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing if applicable
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
