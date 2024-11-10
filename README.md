# Beginner Tutorials - ROS 2 Publisher and Subscriber

This package demonstrates the basic functionality of a **ROS 2 Publisher** and **Subscriber** using C++. The publisher node publishes a message containing a string to a topic at regular intervals, while the subscriber node listens to that topic and logs the received messages.

### Features
- Publisher node publishes a message with a counter every 500 ms.
- Subscriber node listens to the topic and logs received messages.

## Dependencies

- `rclcpp` package for ROS 2 C++ client library
- `std_msgs` for standard message types (e.g., `std_msgs::msg::String`)
- `ament_cmake` as the build system

### Assumptions
- ROS 2 Humble is installed on your system.
- The workspace is sourced correctly, and the ROS environment is properly set up.

## Build Steps

1. Ensure that you have sourced your ROS 2 environment:
   
   `source /opt/ros/humble/setup.bash`

2. Create your ros2 workspace

    `mkdir -p ~/ros2_ws`
    
3. Navigate to the directory 

    `cd ~/ros2_ws`

4. Clone the repository

    `git clone https://github.com/nazringr/my_beginner_tutorials/tree/ros_pub_sub`
   
5. Install rosdep dependencies before building the package

    `rosdep install -i --from-path src --rosdistro humble -y`

6. Build the package using colcon:

    `colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

3. Source the workspace:

    `source install/setup.bash`


## Run Steps

The publisher will publish messages to the topic, and the subscriber will listen and display those messages.

1. In Terminal 1 (Remember sourcing first)

    `ros2 run beginner_tutorials talker`

2. In Terminal 2 (Remember sourcing first)

    `ros2 run beginner_tutorials listener`


## Linting and Code Quality

1. To run cpplint on the source files:
    cpplint src/beginner_tutorials/src/*.cpp > cpplint_output.txt
    
2. To run clang-tidy on the source files:
    clang-tidy -p build/beginner_tutorials --extra-arg=-std=c++17 src/beginner_tutorials/src/publisher_node.cpp src/beginner_tutorials/src/subscriber_node.cpp
