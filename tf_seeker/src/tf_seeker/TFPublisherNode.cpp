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


#include <random>

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rclcpp/rclcpp.hpp"

#include "tf_seeker/TFPublisherNode.hpp"


namespace tf_seeker
{

using namespace std::chrono_literals;

TFPublisherNode::TFPublisherNode()
: Node("tf_producer"),
  rd_(),
  generator_(rd_())
{
  timer_generate_ = create_wall_timer(
    10s, std::bind(&TFPublisherNode::generate_tf, this));
  timer_publish_ = create_wall_timer(
    50ms, std::bind(&TFPublisherNode::publish_tf, this));

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  generate_tf();
}

void
TFPublisherNode::generate_tf()
{
  std::uniform_real_distribution<double> pos_x(-5.0, 5.0);
  std::uniform_real_distribution<double> pos_y(-5.0, 5.0);

  transform_.header.stamp = now();
  transform_.header.frame_id = "odom";
  transform_.child_frame_id = "target";

  transform_.transform.translation.x = pos_x(generator_);
  transform_.transform.translation.y = pos_y(generator_);
  transform_.transform.translation.z = 0.0;
}

void
TFPublisherNode::publish_tf()
{
  transform_.header.stamp = now();
  tf_broadcaster_->sendTransform(transform_);
}

}  //  namespace tf_seeker
