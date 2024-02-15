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

#ifndef COMMS__ACTION_CLIENT_HPP_
#define COMMS__ACTION_CLIENT_HPP_

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace comms
{

class ActionClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  ActionClient();

  void send_request(NavigateToPose::Goal goal);

  bool is_action_finished() {return finished_;}
  bool is_result_success() {return success_;}

protected:
  virtual void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle);
  virtual void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  virtual void result_callback(const GoalHandleNavigateToPose::WrappedResult & result);

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  bool finished_ {false};
  bool success_ {false};
};

}  // namespace comms

#endif  // COMMS__ACTION_CLIENT_HPP_
