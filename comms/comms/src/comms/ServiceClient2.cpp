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

#include "comms/ServiceClient2.hpp"
#include "rclcpp/rclcpp.hpp"

namespace comms
{

using namespace std::chrono_literals;
using std::placeholders::_1;

ServiceClient2::ServiceClient2()
: Node("nav2_service_client")
{
}

using ServiceFutureAndRequestId =
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFutureWithRequestAndRequestId;

ServiceFutureAndRequestId
ServiceClient2::call_client(const std::string & map_url)
{
  service_client_ = create_client<nav2_msgs::srv::LoadMap>("nav2_service");

  while (!service_client_->wait_for_service(1s)) {
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = map_url;

  return service_client_->async_send_request(
    request, std::bind(&ServiceClient2::callback_response, this, _1));
}


using ServiceResponseFuture =
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFutureWithRequest;

void
ServiceClient2::callback_response(ServiceResponseFuture response)
{
  auto request_response_pair = response.get();
  const auto & nav_result = request_response_pair.second->result;

  if (nav_result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS) {
    std::cout << "Result 2: Success" << std::endl;
  } else {
    std::cout << "Result 2: Map does not exist" << std::endl;
  }
}

}  // namespace comms
