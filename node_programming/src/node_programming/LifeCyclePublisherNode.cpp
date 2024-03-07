// Copyright 2024 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "node_programming/LifeCyclePublisherNode.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace node_programming
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

LifeCyclePublisherNode::LifeCyclePublisherNode()
: LifecycleNode("publisher_node")
{
  publisher_ = create_publisher<std_msgs::msg::Int32>("int_topic", 10);
}


CallbackReturn
LifeCyclePublisherNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Configuring...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifeCyclePublisherNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Activating...");

  timer_ = create_wall_timer(
    100ms, std::bind(&LifeCyclePublisherNode::timer_callback, this));

  publisher_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifeCyclePublisherNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Deactivating...");

  timer_ = nullptr;
  publisher_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifeCyclePublisherNode::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Cleaning Up...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifeCyclePublisherNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Shutting Down...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn
LifeCyclePublisherNode::on_error(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  RCLCPP_INFO(get_logger(), "Error State");

  return CallbackReturn::SUCCESS;
}


void
LifeCyclePublisherNode::timer_callback()
{
  message_.data += 1;
  publisher_->publish(message_);
}

}  //  namespace node_programming
