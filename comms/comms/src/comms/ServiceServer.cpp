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


#include "comms/ServiceServer.hpp"

#include "nav2_msgs/srv/load_map.hpp"

#include "rclcpp/rclcpp.hpp"

namespace comms
{

using std::placeholders::_1;
using std::placeholders::_2;

ServiceServer::ServiceServer()
: Node("nav2_service_server")
{
  service_ = create_service<nav2_msgs::srv::LoadMap>(
    "nav2_service",
    std::bind(&ServiceServer::load_map, this, _1, _2));
}

void
ServiceServer::load_map(
  const nav2_msgs::srv::LoadMap::Request::SharedPtr request,
  nav2_msgs::srv::LoadMap::Response::SharedPtr response)
{
  RCLCPP_INFO(get_logger(), "Requesting to load map: %s", request->map_url.c_str());

  if (request->map_url == "") {
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
  } else {
    response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
  }
}

}  // namespace comms
