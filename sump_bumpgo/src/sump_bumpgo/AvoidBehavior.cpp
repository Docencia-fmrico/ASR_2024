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

#include "sump_bumpgo/AvoidBehavior.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace sump_bumpgo
{

using namespace std::chrono_literals;  // NOLINT

AvoidBehavior::AvoidBehavior()
: CascadeLifecycleNode("avoid_behavior"),
  state_(BACK)
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AvoidBehavior::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = create_wall_timer(50ms, std::bind(&AvoidBehavior::control_cycle, this));
  vel_pub_->on_activate();

  go_state(BACK);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AvoidBehavior::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = nullptr;
  vel_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
AvoidBehavior::control_cycle()
{
  RCLCPP_DEBUG(get_logger(), "Avoid Behavior");

  geometry_msgs::msg::Twist out_vel;

  switch (state_) {
    case BACK:
      out_vel.linear.x = -SPEED_LINEAR;

      if (check_back_2_turn()) {
        go_state(TURN);
      }
      break;
    case TURN:
      out_vel.angular.z = SPEED_LINEAR;
      break;
  }

  vel_pub_->publish(out_vel);
}

void
AvoidBehavior::go_state(int new_state)
{
  state_ = new_state;
  state_ts_ = now();
}

bool
AvoidBehavior::check_back_2_turn()
{
  return (now() - state_ts_) > BACK_TIME;
}

}  // namespace sump_bumpgo
