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


#ifndef TF_SEEKER__TFPUBLISHERNODE_HPP_
#define TF_SEEKER__TFPUBLISHERNODE_HPP_

#include <random>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

namespace tf_seeker
{

class TFPublisherNode : public rclcpp::Node
{
public:
  TFPublisherNode();

private:
  void generate_tf();
  void publish_tf();

  rclcpp::TimerBase::SharedPtr timer_generate_;
  rclcpp::TimerBase::SharedPtr timer_publish_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_;

  std::random_device rd_;
  std::mt19937 generator_;
};

}  //  namespace tf_seeker

#endif  // TF_SEEKER__TFPUBLISHERNODE_HPP_
