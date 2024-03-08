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


#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf_seeker/TFSeekerNode.hpp"


namespace tf_seeker
{

using namespace std::chrono_literals;

TFSeekerNode::TFSeekerNode()
: Node("tf_seeker"),
  tf_buffer_(),
  tf_listener_(tf_buffer_),
  vlin_pid_(0.0, 1.0, 0.0, 0.7),
  vrot_pid_(0.0, 1.0, 0.3, 1.0)
{
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  timer_ = create_wall_timer(
    50ms, std::bind(&TFSeekerNode::control_cycle, this));
}

void
TFSeekerNode::control_cycle()
{
  tf2::Stamped<tf2::Transform> bf2target;
  std::string error;

  if (tf_buffer_.canTransform("base_footprint", "target", tf2::TimePointZero, &error)) {
    auto bf2target_msg = tf_buffer_.lookupTransform(
      "base_footprint", "target", tf2::TimePointZero);

    tf2::fromMsg(bf2target_msg, bf2target);

    double x = bf2target.getOrigin().x();
    double y = bf2target.getOrigin().y();

    double angle = atan2(y, x);
    double dist = sqrt(x * x + y * y);

    double vel_rot = std::clamp(vrot_pid_.get_output(angle), -2.0, 2.0);
    double vel_lin = std::clamp(vlin_pid_.get_output(dist - 1.0), -1.0, 1.0);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = vel_lin;
    twist.angular.z = vel_rot;

    vel_publisher_->publish(twist);

    if (fabs(angle) < 0.2 && dist < 1.3) {
      RCLCPP_INFO(get_logger(), "Pew Pew Madafakas");
    }
  } else {
    RCLCPP_WARN_STREAM(get_logger(), "Error in TF odom -> base_footprint [<< " << error << "]");
  }
}

}  //  namespace tf_seeker
