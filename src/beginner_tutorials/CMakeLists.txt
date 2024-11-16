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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(rosbag2 REQUIRED)

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
ament_target_dependencies(talker rclcpp std_msgs rosidl_default_runtime tf2 tf2_ros geometry_msgs)
target_link_libraries(talker "${cpp_typesupport_target}")

# Build listener - subscriber
add_executable(listener src/subscriber_node.cpp)
ament_target_dependencies(listener rclcpp std_msgs rosidl_default_runtime geometry_msgs rosidl_default_generators)
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
  # disable these built-in tests
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
    # build the integration test
  ###########################################################################
  # 1.) BUILD AND INSTALL THE INTEGRATION TEST NODE (aka Node that performs the test)  ##
  ###########################################################################
  add_executable(integration_test_node
    test/integration_test_node.cpp
  )
  target_link_libraries(integration_test_node
    catch_ros2::catch_ros2_with_node_main
    "${cpp_typesupport_target}"
  )
  ament_target_dependencies(integration_test_node
    rclcpp
    std_msgs
    rosidl_default_runtime
  )
  install(TARGETS
    integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )
  ######################################
  ## 2.) ADD INTEGRATION TEST SUPPORT ##
  ######################################
  catch_ros2_add_integration_test (Catch2IntegrationTest
    LAUNCH_FILE integration_test.launch.yaml

    )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
