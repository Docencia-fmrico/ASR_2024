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

#include "node_programming/SubscriberNode.hpp"
#include "node_programming/PublisherNode.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto publisher_node = node_programming::PublisherNode::make_shared();
  auto subscriber_node = node_programming::SubscriberNode::make_shared();

  rclcpp::executors::StaticSingleThreadedExecutor executor;

  executor.add_node(publisher_node);
  executor.add_node(subscriber_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
