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

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"

#include "bt_test_utils.hpp"

using namespace std::chrono_literals;

class StoreWP : public BT::ActionNodeBase
{
public:
  explicit StoreWP(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf)
  {
  }

  void halt() override
  {
  }

  BT::NodeStatus tick() override
  {
    waypoints_.push_back(getInput<geometry_msgs::msg::PoseStamped>("in").value());
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({BT::InputPort<geometry_msgs::msg::PoseStamped>("in")});
  }

  inline static std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
};

TEST(navigation_contract, move_btn_success)
{
  auto node = rclcpp::Node::make_shared("move_btn_node");
  auto nav2_fake_node = std::make_shared<bt_patrolling_tests::Nav2FakeServer>();

  nav2_fake_node->start_server();

  std::atomic_bool finish{false};
  std::thread t([&]() {
      while (!finish.load()) {rclcpp::spin_some(nav2_fake_node);}
    });

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("move_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Move    name="move" goal="{goal}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  geometry_msgs::msg::PoseStamped goal;
  blackboard->set("goal", goal);

  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  while (!finish.load() && rclcpp::ok()) {
    finish.store(tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS);
    rate.sleep();
  }

  finish.store(true);
  t.join();
}

TEST(navigation_contract, move_btn_failure)
{
  auto node = rclcpp::Node::make_shared("move_btn_failure_node");
  auto nav2_abort_node = std::make_shared<bt_patrolling_tests::Nav2AbortServer>();

  nav2_abort_node->start_server();

  std::atomic_bool finish{false};
  std::thread t([&]() {
      while (!finish.load()) {rclcpp::spin_some(nav2_abort_node);}
    });

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("move_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Move    name="move" goal="{goal}"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  geometry_msgs::msg::PoseStamped goal;
  blackboard->set("goal", goal);

  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  auto start = node->now();
  rclcpp::Rate rate(10);
  while (rclcpp::ok() && (node->now() - start) < 5s && status == BT::NodeStatus::RUNNING) {
    status = tree.rootNode()->executeTick();
    rate.sleep();
  }

  EXPECT_EQ(status, BT::NodeStatus::FAILURE);

  finish.store(true);
  t.join();
}

TEST(navigation_contract, required_waypoints_are_visited)
{
  auto node = rclcpp::Node::make_shared("get_waypoint_btn_node");

  rclcpp::spin_some(node);

  {
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName("get_waypoint_bt_node"));

    std::string xml_bt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <GetWaypoint    name="recharge" wp_id="recharge" waypoint="{waypoint}"/>
        </BehaviorTree>
      </root>)";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    rclcpp::Rate rate(10);

    bool finish = false;
    int counter = 0;
    while (!finish && rclcpp::ok()) {
      finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
      counter++;
      rate.sleep();
    }

    auto point = blackboard->get<geometry_msgs::msg::PoseStamped>("waypoint");

    ASSERT_EQ(counter, 1);
    ASSERT_NEAR(point.pose.position.x, 3.67, 0.0000001);
    ASSERT_NEAR(point.pose.position.y, -0.24, 0.0000001);
  }

  {
    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    StoreWP::waypoints_.clear();
    factory.registerNodeType<StoreWP>("StoreWP");
    factory.registerFromPlugin(loader.getOSName("get_waypoint_bt_node"));

    std::string xml_bt =
      R"(
      <root main_tree_to_execute = "MainTree" >
        <BehaviorTree ID="MainTree">
          <Sequence name="root_sequence">
             <GetWaypoint    name="wp1" wp_id="next" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp2" wp_id="next" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp3" wp_id="" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp4" wp_id="recharge" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp5" wp_id="wp1" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wp6" wp_id="wp2" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
             <GetWaypoint    name="wpt" waypoint="{waypoint}"/>
             <StoreWP in="{waypoint}"/>
          </Sequence>
        </BehaviorTree>
      </root>)";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);

    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    rclcpp::Rate rate(10);

    bool finish = false;
    while (!finish && rclcpp::ok()) {
      finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
      rate.sleep();
    }

    const auto & waypoints = StoreWP::waypoints_;
    ASSERT_EQ(waypoints.size(), 7);
    ASSERT_NEAR(waypoints[0].pose.position.x, 1.07, 0.0000001);
    ASSERT_NEAR(waypoints[0].pose.position.y, -12.38, 0.0000001);
    ASSERT_NEAR(waypoints[1].pose.position.x, -5.32, 0.0000001);
    ASSERT_NEAR(waypoints[1].pose.position.y, -8.85, 0.0000001);
    ASSERT_NEAR(waypoints[2].pose.position.x, -0.56, 0.0000001);
    ASSERT_NEAR(waypoints[2].pose.position.y, 0.24, 0.0000001);
    ASSERT_NEAR(waypoints[3].pose.position.x, 3.67, 0.0000001);
    ASSERT_NEAR(waypoints[3].pose.position.y, -0.24, 0.0000001);
    ASSERT_NEAR(waypoints[4].pose.position.x, 1.07, 0.0000001);
    ASSERT_NEAR(waypoints[4].pose.position.y, -12.38, 0.0000001);
    ASSERT_NEAR(waypoints[5].pose.position.x, -5.32, 0.0000001);
    ASSERT_NEAR(waypoints[5].pose.position.y, -8.85, 0.0000001);
    ASSERT_NEAR(waypoints[6].pose.position.x, -0.56, 0.0000001);
    ASSERT_NEAR(waypoints[6].pose.position.y, 0.24, 0.0000001);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
