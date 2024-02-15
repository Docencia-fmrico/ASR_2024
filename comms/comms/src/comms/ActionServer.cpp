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


#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "comms/ActionServer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace comms
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

ActionServer::ActionServer()
: Node("nav2_action_server")
{
}

void
ActionServer::start_server()
{
  navigate_action_server_ = rclcpp_action::create_server<NavigateToPose>(
    shared_from_this(),
    "navigate_to_pose",
    std::bind(&ActionServer::handle_goal, this, _1, _2),
    std::bind(&ActionServer::handle_cancel, this, _1),
    std::bind(&ActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(get_logger(), "Action Server Ready.");
}

rclcpp_action::GoalResponse
ActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ActionServer::handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

  timer_ = nullptr;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void
ActionServer::handle_accepted(std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  auto pose_cmd = goal_handle->get_goal()->pose.pose;
  tf2::Quaternion q;
  tf2::fromMsg(pose_cmd.orientation, q);

  RCLCPP_INFO(
    get_logger(), "Starting navigation to %lf, %lf, %lf",
    pose_cmd.position.x, pose_cmd.position.y, q.getAngle());

  goal_handle_ = goal_handle;
  current_times_ = 0;
  timer_ = create_wall_timer(
    1s, std::bind(&ActionServer::execute, this));
}

void
ActionServer::execute()
{
  if (current_times_ < 10 && !goal_handle_->is_canceling()) {
    auto feedback = std::make_shared<NavigateToPose::Feedback>();

    feedback->distance_remaining = 10.0 - current_times_;
    goal_handle_->publish_feedback(feedback);
  } else {
    auto result = std::make_shared<NavigateToPose::Result>();

    if (goal_handle_->is_canceling()) {
      goal_handle_->canceled(result);

      RCLCPP_INFO(get_logger(), "Action Canceled");
    } else if (current_times_ >= 10) {
      goal_handle_->succeed(result);
      RCLCPP_INFO(get_logger(), "Navigation Succeeded");
    }

    timer_ = nullptr;
  }
  current_times_++;
}

}  // namespace comms
