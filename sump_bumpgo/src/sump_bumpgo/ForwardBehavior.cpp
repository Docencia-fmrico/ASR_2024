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

#include "sump_bumpgo/ForwardBehavior.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace sump_bumpgo
{

using namespace std::chrono_literals;  // NOLINT

ForwardBehavior::ForwardBehavior()
: CascadeLifecycleNode("forward_behavior")
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForwardBehavior::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = create_wall_timer(50ms, std::bind(&ForwardBehavior::control_cycle, this));
  vel_pub_->on_activate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForwardBehavior::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  timer_ = nullptr;
  vel_pub_->on_deactivate();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void
ForwardBehavior::control_cycle()
{
  RCLCPP_INFO(get_logger(), "Forward Behavior");

  geometry_msgs::msg::Twist out_vel;
  out_vel.linear.x = SPEED_LINEAR;

  vel_pub_->publish(out_vel);
}

}  // namespace sump_bumpgo
