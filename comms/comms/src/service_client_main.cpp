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

#include "comms/ServiceClient.hpp"
#include "comms/ServiceClient2.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usa: ros2 run comms service_client_main file_name" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  {  // Widely used approach
    auto node = std::make_shared<comms::ServiceClient>();

    auto result = node->call_client(argv[1]);
    if (result.has_value()) {
      const auto & nav_result = result.value().result;
      if (nav_result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
        std::cout << "Result 1: Success" << std::endl;
      } else {
        std::cout << "Result 1: Map does not exist" << std::endl;
      }
    }
  }

  {  // Better approach
    auto node = std::make_shared<comms::ServiceClient2>();
    auto result = node->call_client(argv[1]);

    if (rclcpp::spin_until_future_complete(node, result) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    }
  }

  rclcpp::shutdown();

  return 0;
}
