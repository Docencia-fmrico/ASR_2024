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

#ifndef COMMS__SERVICE_CLIENT2_HPP_
#define COMMS__SERVICE_CLIENT2_HPP_

#include "nav2_msgs/srv/load_map.hpp"

#include "rclcpp/rclcpp.hpp"

namespace comms
{

using ServiceResponseFuture =
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFutureWithRequest;
using ServiceFutureAndRequestId =
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFutureWithRequestAndRequestId;

class ServiceClient2 : public rclcpp::Node
{
public:
  ServiceClient2();

  ServiceFutureAndRequestId call_client(const std::string & map_url);
  void callback_response(ServiceResponseFuture response);

private:
  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr service_client_;
};

}  // namespace comms

#endif  // COMMS__SERVICE_CLIENT2_HPP_
