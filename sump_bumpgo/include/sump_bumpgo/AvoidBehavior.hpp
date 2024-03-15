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

#ifndef BUMP_BUMPGO__AVOIDBEHAVIOR_HPP_
#define BUMP_BUMPGO__AVOIDBEHAVIOR_HPP_

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace sump_bumpgo
{

using namespace std::chrono_literals;  // NOLINT

class AvoidBehavior : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  AvoidBehavior();

private:
  void control_cycle();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  static const int BACK = 0;
  static const int TURN = 1;

  int state_;
  rclcpp::Time state_ts_;

  void go_state(int new_state);
  bool check_back_2_turn();

  const rclcpp::Duration TURN_TIME {2s};
  const rclcpp::Duration BACK_TIME {2s};
  static constexpr float SPEED_ANGULAR = 0.5f;
  static constexpr float SPEED_LINEAR = 0.3f;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

}  // namespace sump_bumpgo

#endif  // BUMP_BUMPGO__BUMPGOBEHAVIOR_HPP_
