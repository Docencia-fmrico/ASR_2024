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

#ifndef COMMS__SERVICE_CLIENT_HPP_
#define COMMS__SERVICE_CLIENT_HPP_

#include "nav2_msgs/srv/load_map.hpp"

#include "rclcpp/rclcpp.hpp"

namespace comms
{

class ServiceClient : public rclcpp::Node
{
public:
  ServiceClient();

  std::optional<nav2_msgs::srv::LoadMap::Response> call_client(const std::string & map_url);
};

}  // namespace comms

#endif  // COMMS__SERVICE_CLIENT_HPP_
