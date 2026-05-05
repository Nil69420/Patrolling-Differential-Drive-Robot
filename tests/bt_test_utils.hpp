// Copyright 2026 Nil
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

#ifndef TESTS__BT_TEST_UTILS_HPP_
#define TESTS__BT_TEST_UTILS_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <list>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace bt_patrolling_tests
{
using std::chrono_literals::operator""s;
using std::placeholders::_1;
using std::placeholders::_2;

class VelocitySinkNode : public rclcpp::Node
{
public:
  VelocitySinkNode()
  : Node("VelocitySink")
  {
    vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/output_vel", 100, std::bind(&VelocitySinkNode::vel_callback, this, _1));
  }

  void vel_callback(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    vel_msgs_.push_back(*msg);
  }

  std::list<geometry_msgs::msg::Twist> vel_msgs_;

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
};

class Nav2FakeServer : public rclcpp::Node
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

public:
  Nav2FakeServer()
  : Node("nav2_fake_server_node")
  {
  }

  ~Nav2FakeServer()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void start_server()
  {
    move_action_server_ = rclcpp_action::create_server<NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose",
      std::bind(&Nav2FakeServer::handle_goal, this, _1, _2),
      std::bind(&Nav2FakeServer::handle_cancel, this, _1),
      std::bind(&Nav2FakeServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr move_action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread(std::bind(&Nav2FakeServer::execute, this, _1), goal_handle);
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    auto result = std::make_shared<NavigateToPose::Result>();

    auto start = now();

    while ((now() - start) < 5s) {
      feedback->distance_remaining = 5.0 - (now() - start).seconds();
      goal_handle->publish_feedback(feedback);
    }

    goal_handle->succeed(result);
  }

  std::thread worker_;
};

class Nav2AbortServer : public rclcpp::Node
{
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

public:
  Nav2AbortServer()
  : Node("nav2_abort_server_node")
  {
  }

  ~Nav2AbortServer()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

  void start_server()
  {
    move_action_server_ = rclcpp_action::create_server<NavigateToPose>(
      shared_from_this(),
      "navigate_to_pose",
      std::bind(&Nav2AbortServer::handle_goal, this, _1, _2),
      std::bind(&Nav2AbortServer::handle_cancel, this, _1),
      std::bind(&Nav2AbortServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<NavigateToPose>::SharedPtr move_action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const NavigateToPose::Goal>)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateToPose>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    if (worker_.joinable()) {
      worker_.join();
    }
    worker_ = std::thread(std::bind(&Nav2AbortServer::execute, this, _1), goal_handle);
  }

  void execute(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    auto result = std::make_shared<NavigateToPose::Result>();
    goal_handle->abort(result);
  }

  std::thread worker_;
};

}  // namespace bt_patrolling_tests

#endif  // TESTS__BT_TEST_UTILS_HPP_
