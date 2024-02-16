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

#ifndef FSM_BUMPGO__BUMPGONODE_HPP_
#define FSM_BUMPGO__BUMPGONODE_HPP_

#include "kobuki_ros_interfaces/msg/bumper_event.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

namespace fsm_bumpgo
{

using namespace std::chrono_literals;  // NOLINT

class BumpGoNode : public rclcpp::Node
{
public:
  BumpGoNode();

private:
  void bumper_callback(kobuki_ros_interfaces::msg::BumperEvent::UniquePtr msg);
  void control_cycle();

  static const int FORWARD = 0;
  static const int BACK = 1;
  static const int TURN = 2;
  int state_;
  rclcpp::Time state_ts_;

  void go_state(int new_state);
  bool check_forward_2_back();
  bool check_back_2_turn();
  bool check_turn_2_forward();

  const rclcpp::Duration TURNING_TIME {2s};
  const rclcpp::Duration BACKING_TIME {2s};

  static constexpr float SPEED_LINEAR = 0.3f;
  static constexpr float SPEED_ANGULAR = 0.3f;

  rclcpp::Subscription<kobuki_ros_interfaces::msg::BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  kobuki_ros_interfaces::msg::BumperEvent::UniquePtr last_bump_;
};

}  // namespace fsm_bumpgo

#endif  // FSM_BUMPGO__BUMPGONODE_HPP_
