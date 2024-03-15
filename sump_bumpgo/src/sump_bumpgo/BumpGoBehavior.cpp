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

#include <utility>
#include "sump_bumpgo/BumpGoBehavior.hpp"

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace sump_bumpgo
{

using namespace std::chrono_literals;
using std::placeholders::_1;

BumpGoBehavior::BumpGoBehavior()
: CascadeLifecycleNode("bumpgo_behavior"),
  state_(FORWARD)
{
  bumper_sub_ = create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
    "/events/bumper", 10, std::bind(&BumpGoBehavior::bumper_callback, this, _1));
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BumpGoBehavior::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = create_wall_timer(50ms, std::bind(&BumpGoBehavior::control_cycle, this));

  state_ts_ = now();

  last_bump_ = std::make_unique<kobuki_ros_interfaces::msg::BumperEvent>();
  last_bump_->state = kobuki_ros_interfaces::msg::BumperEvent::RELEASED;

  go_state(FORWARD);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BumpGoBehavior::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = nullptr;

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
BumpGoBehavior::bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg)
{
  last_bump_ = std::move(msg);
}

void
BumpGoBehavior::control_cycle()
{
  switch (state_) {
    case FORWARD:
      RCLCPP_INFO(get_logger(), "BumpGo Behavior: FORWARD");
      add_activation("forward_behavior");

      if (check_forward_2_avoid()) {
        go_state(AVOID);
      }
      break;
    case AVOID:
      RCLCPP_INFO(get_logger(), "BumpGo Behavior: AVOID");
      add_activation("avoid_behavior");

      if (check_avoid_2_forward()) {
        go_state(FORWARD);
      }
      break;
  }
}

void
BumpGoBehavior::go_state(int new_state)
{
  clear_activation();

  state_ = new_state;
  state_ts_ = now();
}

bool
BumpGoBehavior::check_forward_2_avoid()
{
  return last_bump_->state == kobuki_ros_interfaces::msg::BumperEvent::PRESSED;
}

bool
BumpGoBehavior::check_avoid_2_forward()
{
  return (now() - state_ts_) > AVOID_TIME;
}

}  // namespace sump_bumpgo
