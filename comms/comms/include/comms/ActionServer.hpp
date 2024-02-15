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

#ifndef COMMS__ACTION_SERVER_HPP_
#define COMMS__ACTION_SERVER_HPP_

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace comms
{

class ActionServer : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  ActionServer();
  void start_server();

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr navigate_action_server_;
  NavigateToPose::Goal current_goal_;
  std::shared_ptr<GoalHandleNavigateToPose> goal_handle_;
  rclcpp::TimerBase::SharedPtr timer_;
  int current_times_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandleNavigateToPose> goal_handle);

  void execute();
};

}  // namespace comms

#endif  // COMMS__ACTION_SERVER_HPP_
