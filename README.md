# Beginner Tutorials - Services, Logging, and Launch files

## Overview

In this branch, beginner_tutorials package has been modified to include new features for ROS 2 services, logging, and launch files. 

## Dependencies

- `rclcpp`: ROS 2 C++ client library for developing ROS 2 nodes.
- `std_msgs`: Provides standard message types such as `std_msgs::msg::String`.
- `ament_cmake`: Build system used to compile and package the ROS 2 package.
- `rosidl_default_generators`: Generates code for custom services and messages in ROS 2.
- `rosidl_default_runtime`: Provides runtime support for custom services and messages.
- `ros2launch`: A tool to launch multiple ROS 2 nodes using launch files.
- `ament_lint_auto`: Automatically detects linting dependencies for code quality checks.
- `ament_lint_common`: Common linting utilities for maintaining code quality in ROS 2 packages.


## Assumptions
- ROS 2 Humble is installed on your system.
- The workspace is sourced correctly, and the ROS environment is properly set up.


## Build Steps

```bash
# 1. Ensure that you have sourced your ROS 2 environment
# This step sets up your ROS 2 environment for usage
source /opt/ros/humble/setup.bash

# 2. Create your ROS 2 workspace
# The workspace will be where the package and dependencies are stored
mkdir -p ~/ros2_ws

# 3. Navigate to the workspace directory
# Change to the directory where you will work on the package
cd ~/ros2_ws

# 4. Clone the repository
# Clone the desired branch of the repository to your workspace
git clone https://github.com/nazringr/my_beginner_tutorials.git

# 5. Install rosdep dependencies before building the package
# rosdep installs the necessary dependencies for the package
rosdep install -i --from-path src --rosdistro humble -y

# 6. Build the package using colcon
# This will build the package, including setting up the compile commands for IDEs
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# 7. Source the workspace to set up the environment variables
# This step ensures that the workspace is correctly sourced and ready to use
source install/setup.bash
```

## Run Steps

The publisher will publish messages to the topic, and the subscriber will listen and display those messages.

1. In Terminal 1, Run Talker (Remember sourcing first)

    `ros2 run beginner_tutorials talker`

2. In Terminal 2, Run Listener (Remember sourcing first)

    `ros2 run beginner_tutorials listener`

3. In Terminal 3, Call the Service (Remember sourcing first)
    
    `ros2 service call /set_string beginner_tutorials/srv/SetString "{input_string: Nazrin}"`

or

1. In Terminal 1, Run Launch file (Remember sourcing first)
    
    `ros2 launch beginner_tutorials launch.py publish_frequency:=1000`

2. In Terminal 2, Call the Service (Remember sourcing first)
    
    `ros2 service call /set_string beginner_tutorials/srv/SetString "{input_string: Nazrin}"`


    
## Linting and Code Quality

1. To run cpplint on the source files:
    cpplint src/beginner_tutorials/src/*.cpp > cpplint_output.txt

2. To run clang-tidy on the source files:
    clang-tidy -p build/beginner_tutorials --extra-arg=-std=c++17 src/beginner_tutorials/src/publisher_node.cpp src/beginner_tutorials/src/subscriber_node.cpp
