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

#ifndef NODE_PROGRAMMING__SUBSCRIBERNODE_HPP_
#define NODE_PROGRAMMING__SUBSCRIBERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "std_msgs/msg/int32.hpp"

namespace node_programming
{

class SubscriberNode : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SubscriberNode)

  SubscriberNode();
  void callback(const std_msgs::msg::Int32::SharedPtr msg);

private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_;
};

}  //  namespace node_programming
#endif  // NODE_PROGRAMMING__SUBSCRIBERNODE_HPP_
