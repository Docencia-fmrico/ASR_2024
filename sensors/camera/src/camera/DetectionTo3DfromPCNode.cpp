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

#include "camera/DetectionTo3DfromPCNode.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "rclcpp/rclcpp.hpp"

namespace camera
{

using std::placeholders::_1;
using std::placeholders::_2;

DetectionTo3DfromPCNode::DetectionTo3DfromPCNode()
: Node("detection_to_3d_from_pc2_node")
{
  pc2_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
    this, "input_pointcloud", rclcpp::SensorDataQoS().get_rmw_qos_profile());
  detection_sub_ =
    std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(
    this, "input_detection_2d", rclcpp::SensorDataQoS().reliable().get_rmw_qos_profile());
  sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
    MySyncPolicy(10), *pc2_sub_, *detection_sub_);
  sync_->registerCallback(std::bind(&DetectionTo3DfromPCNode::callback_sync, this, _1, _2));

  detection_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
    "output_detection_3d", rclcpp::SensorDataQoS().reliable());
}

void
DetectionTo3DfromPCNode::callback_sync(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pc_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg)
{
  if (detection_pub_->get_subscription_count() > 0) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc_msg, *pc);

    vision_msgs::msg::Detection3DArray detections_3d_msg;
    detections_3d_msg.header = detection_msg->header;

    for (const auto & detection : detection_msg->detections) {
      vision_msgs::msg::Detection3D detection_3d_msg;
      detection_3d_msg.header = detection_msg->header;
      detection_3d_msg.results = detection.results;

      pcl::PointXYZ & center = pc->at(
        detection.bbox.center.position.x,
        detection.bbox.center.position.y);

      detection_3d_msg.bbox.center.position.x = center.x;
      detection_3d_msg.bbox.center.position.y = center.y;
      detection_3d_msg.bbox.center.position.z = center.z;

      if (!std::isnan(center.x) && !std::isinf(center.x)) {
        detections_3d_msg.detections.push_back(detection_3d_msg);
      }
    }

    if (!detections_3d_msg.detections.empty()) {
      detection_pub_->publish(detections_3d_msg);
    }
  }
}

}  // namespace camera
