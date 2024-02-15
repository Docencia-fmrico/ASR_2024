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

#include <memory>

#include "comms/ActionClient.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cerr << "Usa: ros2 run comms action_client_main x y" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<comms::ActionClient>();
  auto goal = comms::ActionClient::NavigateToPose::Goal();

  goal.pose.header.frame_id = "map";
  goal.pose.pose.position.x = atof(argv[1]);
  goal.pose.pose.position.y = atof(argv[2]);

  node->send_request(goal);

  rclcpp::Rate rate(10);
  while (rclcpp::ok() && !node->is_action_finished()) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  if (node->is_result_success()) {
    std::cout << "Result: Success" << std::endl;
  } else {
    std::cerr << "Result: error" << std::endl;
  }

  rclcpp::shutdown();

  return 0;
}
