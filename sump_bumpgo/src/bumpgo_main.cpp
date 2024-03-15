// Copyright 2021 Intelligent Robotics Lab
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

#include <memory>

#include "sump_bumpgo/BumpGoBehavior.hpp"
#include "sump_bumpgo/ForwardBehavior.hpp"
#include "sump_bumpgo/AvoidBehavior.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exe(rclcpp::ExecutorOptions(), 3);

  auto bumpgo_node = std::make_shared<sump_bumpgo::BumpGoBehavior>();
  auto forward_node = std::make_shared<sump_bumpgo::ForwardBehavior>();
  auto avoid_node = std::make_shared<sump_bumpgo::AvoidBehavior>();

  exe.add_node(bumpgo_node->get_node_base_interface());
  exe.add_node(forward_node->get_node_base_interface());
  exe.add_node(avoid_node->get_node_base_interface());

  bumpgo_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  forward_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  avoid_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  bumpgo_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
