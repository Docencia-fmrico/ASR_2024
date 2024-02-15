// Copyright 2023 Intelligent Robotics Lab
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


#include "nav2_msgs/srv/load_map.hpp"

#include "comms/ServiceClient.hpp"
#include "rclcpp/rclcpp.hpp"

namespace comms
{

using namespace std::chrono_literals;

ServiceClient::ServiceClient()
: Node("nav2_service_client")
{
}

std::optional<nav2_msgs::srv::LoadMap::Response>
ServiceClient::call_client(const std::string & map_url)
{
  auto node_aux = rclcpp::Node::make_shared("nav2_service_client_aux");
  auto service_client = node_aux->create_client<nav2_msgs::srv::LoadMap>("nav2_service");

  while (!service_client->wait_for_service(1s)) {
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = map_url;

  auto result = service_client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_aux, result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "service call failed :(");
    service_client->remove_pending_request(result);
    return {};
  }

  return *result.get();
}

}  // namespace comms
