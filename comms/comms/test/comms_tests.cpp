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

#include "gtest/gtest.h"

#include "nav2_msgs/srv/load_map.hpp"

#include "comms/ServiceServer.hpp"
#include "comms/ServiceClient.hpp"
#include "comms/ActionServer.hpp"
#include "comms/ActionClient.hpp"

using namespace std::chrono_literals;

TEST(test_comms, service_server)
{
  rclcpp::executors::SingleThreadedExecutor exe;
  auto server_node = std::make_shared<comms::ServiceServer>();
  exe.add_node(server_node);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  auto client_node = rclcpp::Node::make_shared("nav2_service_client");
  auto service_client = client_node->create_client<nav2_msgs::srv::LoadMap>("nav2_service");
  auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();

  {
    request->map_url = "";

    while (!service_client->wait_for_service(1s)) {
      RCLCPP_INFO(client_node->get_logger(), "service not available, waiting again...");
    }

    auto result = service_client->async_send_request(request);

    ASSERT_EQ(
      rclcpp::spin_until_future_complete(client_node, result),
      rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_EQ(result.get()->result, nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST);
  }

  {
    request->map_url = "file://map.yaml";

    while (!service_client->wait_for_service(1s)) {
      RCLCPP_INFO(client_node->get_logger(), "service not available, waiting again...");
    }

    auto result = service_client->async_send_request(request);

    ASSERT_EQ(
      rclcpp::spin_until_future_complete(client_node, result),
      rclcpp::FutureReturnCode::SUCCESS);
    ASSERT_EQ(result.get()->result, nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS);
  }

  finish = true;
  t.join();
}

TEST(test_comms, service_client)
{
  rclcpp::executors::SingleThreadedExecutor exe;
  auto server_node = std::make_shared<comms::ServiceServer>();
  auto server_client = std::make_shared<comms::ServiceClient>();
  exe.add_node(server_node);

  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  {
    auto result = server_client->call_client("");
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().result, nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST);
  }

  {
    auto result = server_client->call_client("file://map.yaml");
    ASSERT_TRUE(result.has_value());
    ASSERT_EQ(result.value().result, nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS);
  }

  finish = true;
  t.join();
}


TEST(test_comms, action_server)
{
  rclcpp::executors::SingleThreadedExecutor exe;
  auto server_node = std::make_shared<comms::ActionServer>();
  auto client_node = rclcpp::Node::make_shared("action_client");
  exe.add_node(server_node);
  exe.add_node(client_node);

  server_node->start_server();


  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  using GoalHandleNavigateToPose =
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;

  auto action_client = rclcpp_action::create_client<comms::ActionServer::NavigateToPose>(
    client_node.get(), "navigate_to_pose");

  ASSERT_TRUE(action_client->wait_for_action_server());
  auto goal_msg = comms::ActionServer::NavigateToPose::Goal();

  auto send_goal_options =
    rclcpp_action::Client<comms::ActionServer::NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
      ASSERT_TRUE(goal_handle);
    };

  send_goal_options.result_callback =
    [&finish](const GoalHandleNavigateToPose::WrappedResult & result)
    {
      ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
      finish = true;
    };

  action_client->async_send_goal(goal_msg, send_goal_options);

  while (!finish) {}

  t.join();
}

TEST(test_comms, action_client)
{
  rclcpp::executors::SingleThreadedExecutor exe;
  auto server_node = std::make_shared<comms::ActionServer>();
  auto client_node = std::make_shared<comms::ActionClient>();
  exe.add_node(server_node);
  exe.add_node(client_node);

  server_node->start_server();


  bool finish = false;
  std::thread t([&]() {
      while (!finish) {exe.spin_some();}
    });

  auto goal_msg = comms::ActionServer::NavigateToPose::Goal();

  client_node->send_request(goal_msg);

  rclcpp::Rate rate(10);
  while (rclcpp::ok() && !client_node->is_action_finished()) {
    rate.sleep();
  }

  ASSERT_TRUE(client_node->is_result_success());

  finish = true;
  t.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
