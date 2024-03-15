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

#ifndef BUMP_BUMPGO__BUMPGOBEHAVIOR_HPP_
#define BUMP_BUMPGO__BUMPGOBEHAVIOR_HPP_

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace sump_bumpgo
{

using namespace std::chrono_literals;  // NOLINT

class BumpGoBehavior : public rclcpp_cascade_lifecycle::CascadeLifecycleNode
{
public:
  BumpGoBehavior();

private:
  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);
  void control_cycle();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state);

  static const int FORWARD = 0;
  static const int AVOID = 1;

  int state_;
  rclcpp::Time state_ts_;

  void go_state(int new_state);
  bool check_forward_2_avoid();
  bool check_avoid_2_forward();

  const rclcpp::Duration AVOID_TIME {4s};

  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  kobuki_ros_interfaces::msg::BumperEvent::UniquePtr last_bump_;
};

}  // namespace sump_bumpgo

#endif  // BUMP_BUMPGO__BUMPGOBEHAVIOR_HPP_
