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
 * @file minimal_subscriber.cpp
 * @author Nazrin Gurbanova (nazrin@umd.edu)
 * @brief ROS2 Node that subscribes to a topic and sends the received message to a service.
 * @version 0.1
 * @date 2024-11-8
 * @copyright Copyright (c) 2024
 */

#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "beginner_tutorials/srv/set_string.hpp"

using std::placeholders::_1;

/**
 * @class MinimalSubscriber
 * @brief ROS2 Node that subscribes to a topic and sends the received message to a service.
 *
 * This node listens to messages on the "topic" topic and sends the received string
 * to a service to update the message.
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the MinimalSubscriber node.
   *
   * Initializes the subscription to the "topic" topic and sets up the client
   * for the "set_string" service.
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    // Subscribe to the "topic" topic to receive messages of type String
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    // Create a client to call the "set_string" service
    client_ = this->create_client<beginner_tutorials::srv::SetString>("set_string");
  }

 private:
  /**
   * @brief Callback function for handling incoming messages on the "topic" topic.
   *
   * When a message is received, this function logs the message data and sends
   * the message string to the "set_string" service to update the string.
   *
   * @param msg The received message, a pointer to a String message.
   */
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    // Log the received message
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: '" << msg->data.c_str());

    // Wait for the service to become available
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available!");
      return;
    }

    // Create a request for the SetString service and set the input string
    auto request = std::make_shared<beginner_tutorials::srv::SetString::Request>();
    request->input_string = msg->data;

    // Send the request asynchronously and bind the response callback
    auto future = client_->async_send_request(request,
      std::bind(&MinimalSubscriber::service_response_callback, this, std::placeholders::_1));
  }

  /**
   * @brief Callback function for handling the service response.
   *
   * This function processes the service response and logs the result.
   *
   * @param future The future object representing the service response.
   */
  void service_response_callback(
      rclcpp::Client<beginner_tutorials::srv::SetString>::SharedFuture future) {
    try {
      // Get the result from the service response and log it
      auto result = future.get();
      RCLCPP_INFO_STREAM(this->get_logger(), "Service response: '" << result->status_message);
    } catch (const std::exception &e) {
      // Log error if service call fails
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  /**< Subscription for receiving messages from the topic */
  rclcpp::Client<beginner_tutorials::srv::SetString>::SharedPtr client_; /**< Client for calling the SetString service */
};

/**
 * @brief Main function for initializing and spinning the MinimalSubscriber node.
 *
 * Initializes ROS2, spins the node to keep it alive and processing callbacks,
 * and shuts down ROS2 when finished.
 *
 * @param argc The argument count for ROS2 initialization.
 * @param argv The argument vector for ROS2 initialization.
 * @return 0 on success, non-zero on failure.
 */
int main(int argc, char* argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Spin the node to keep it alive and processing callbacks
  rclcpp::spin(std::make_shared<MinimalSubscriber>());

  // Shut down ROS2 when done
  rclcpp::shutdown();
  return 0;
}
