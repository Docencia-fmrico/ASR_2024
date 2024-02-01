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

#include "node_programming/SubscriberNode.hpp"

using std::placeholders::_1;

namespace node_programming
{

SubscriberNode::SubscriberNode()
: Node("subscriber_node")
{
  subscriber_ = create_subscription<std_msgs::msg::Int32>(
    "int_topic", 10,
    std::bind(&SubscriberNode::callback, this, _1));
}

void
SubscriberNode::callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Hello %d", msg->data);
}

}  //  namespace node_programming
