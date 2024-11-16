// Apache License
// Version 2.0, January 2004
// http://www.apache.org/licenses/LICENSE-2.0
//
// Copyright (c) 2024 Nazrin Gurbanova
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.



/**
 * @file publisher_node.cpp
 * @author Nazrin Gurbanova (nazrin@umd.edu)
 * @brief ROS2 Node that publishes a string message and provides a service to modify the message.
 * @version 0.1
 * @date 2024-11-8
 * @copyright Copyright (c) 2024
 */

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <beginner_tutorials/srv/set_string.hpp>  // Updated service header

/**
 * @class PublisherAndServiceNode
 * @brief ROS2 Talker node that broadcasts a tf frame "/talk" with parent "/world".aa * 
 * This node publishes a string message on a specified frequency and allows users to modify
 * the string via a service.
 */
class PublisherAndServiceNode : public rclcpp::Node {
 public:
  /**
   * @brief Constructor that initializes the publisher and service, and sets up the timer.
   * 
   * Declares parameters, sets up the publisher, service, and timer, and handles frequency validation.
   */
  PublisherAndServiceNode() : Node("publisher_service_node") {
    // Declare the parameter for publish frequency with a default value of 500 ms
    this->declare_parameter("publish_frequency", 500);
    this->message.data = "Use service to change this string";  // Default message

    // Initialize the publisher for sending messages
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // Initialize the service for changing the message string
    service_ = this->create_service<beginner_tutorials::srv::SetString>(
        "set_string",  
        std::bind(&PublisherAndServiceNode::set_string_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Check if the frequency is still at the default of 500ms
    if (this->get_parameter("publish_frequency").as_int() == 500) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Publisher frequency has not changed, is still: "
              << this->get_parameter("publish_frequency").as_int());
    }

    // Check if the frequency is too high and could cause issues
    if (this->get_parameter("publish_frequency").as_int() > 5000) {
      RCLCPP_FATAL_STREAM(
          this->get_logger(),
          "Publisher frequency is set very high and may cause issues!");
      rclcpp::shutdown();
      return;
    }
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    // Set up a timer to publish messages at the specified frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            this->get_parameter("publish_frequency").as_int()),
        std::bind(&PublisherAndServiceNode::timer_callback, this));
  }

 private:
  /**
   * @brief Timer callback function to publish the message periodically.
   * 
   * This function is triggered by the timer and publishes the message to the topic.
   */
  void timer_callback() {
    auto message = this->message;
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    publisher_->publish(message);
    this->make_transforms();
  }

  /**
   * @brief Service callback function to modify the message string.
   * 
   * This service callback updates the string message based on the request input. If the string is empty, 
   * it sends an error response.
   * 
   * @param request The service request containing the new string to set.
   * @param resp The service response containing the status message.
   */
  void set_string_callback(
      const std::shared_ptr<beginner_tutorials::srv::SetString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::SetString::Response> resp) {
    // Check if the incoming string is empty
    if (request->input_string.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Received empty string! Cannot update message.");
      resp->status_message =
          "Failed to change string: Received empty string.";
    } else {
      // Update the message if the string is valid
      this->message.data = request->input_string;
      resp->status_message = request->input_string;
      RCLCPP_DEBUG_STREAM(this->get_logger(),
                          "Received Service Request: " << request->input_string);
    }
  }
    void make_transforms() {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "talk";

        t.transform.translation.x = 0.1;
        t.transform.translation.y = 0.1;
        t.transform.translation.z = 0.1;

        tf2::Quaternion q;
        q.setRPY(0.1, 0.1, 0.1);

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_static_broadcaster_->sendTransform(t);
  }

  std_msgs::msg::String message;  /**< The message to be published */
  rclcpp::TimerBase::SharedPtr timer_;  /**< Timer for controlling publish frequency */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  /**< Publisher for sending messages */
  rclcpp::Service<beginner_tutorials::srv::SetString>::SharedPtr service_;  /**< Service for changing the message */

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

/**
 * @brief Main function for initializing and spinning the PublisherAndServiceNode.
 * 
 * Initializes ROS2, spins the node to keep it running, and shuts it down when finished.
 * 
 * @param argc The argument count for ROS2 initialization.
 * @param argv The argument vector for ROS2 initialization.
 * @return 0 on success, non-zero on failure.
 */
int main(int argc, char* argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Spin the node to keep it alive and processing callbacks
  rclcpp::spin(std::make_shared<PublisherAndServiceNode>());
  
  // Shut down ROS2 when done
  rclcpp::shutdown();
  return 0;
}
