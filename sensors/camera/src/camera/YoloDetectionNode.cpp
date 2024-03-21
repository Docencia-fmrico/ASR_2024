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

#include <memory>

#include "camera/YoloDetectionNode.hpp"

#include "yolov8_msgs/msg/detection_array.hpp"

#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"


#include "rclcpp/rclcpp.hpp"

namespace camera
{

using std::placeholders::_1;

YoloDetectionNode::YoloDetectionNode()
: Node("darkent_detection_node")
{
  detection_sub_ = create_subscription<yolov8_msgs::msg::DetectionArray>(
    "input_detection", rclcpp::SensorDataQoS().reliable(),
    std::bind(&YoloDetectionNode::detection_callback, this, _1));
  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    "output_detection_2d", rclcpp::SensorDataQoS().reliable());
}

void
YoloDetectionNode::detection_callback(
  const yolov8_msgs::msg::DetectionArray::ConstSharedPtr & msg)
{
  vision_msgs::msg::Detection2DArray detection_array_msg;
  detection_array_msg.header = msg->header;

  for (const auto & detection : msg->detections) {
    vision_msgs::msg::Detection2D detection_msg;
    detection_msg.header = msg->header;

    detection_msg.bbox.center.position.x = detection.bbox.center.position.x;
    detection_msg.bbox.center.position.y = detection.bbox.center.position.y;
    detection_msg.bbox.size_x = detection.bbox.size.x;
    detection_msg.bbox.size_y = detection.bbox.size.y;

    vision_msgs::msg::ObjectHypothesisWithPose obj_msg;
    obj_msg.hypothesis.class_id = detection.class_name;
    obj_msg.hypothesis.score = detection.score;

    detection_msg.results.push_back(obj_msg);
    detection_array_msg.detections.push_back(detection_msg);
  }

  detection_pub_->publish(detection_array_msg);
}

}  // namespace camera
