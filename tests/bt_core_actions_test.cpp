#include <memory>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "gtest/gtest.h"

#include "bt_test_utils.hpp"

using namespace std::chrono_literals;

TEST(bt_core_actions, recharge_btn)
{
  auto node = rclcpp::Node::make_shared("recharge_btn_node");

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("recharge_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Recharge    name="recharge"/>
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

  float battery_level;
  ASSERT_TRUE(blackboard->get("battery_level", battery_level));
  ASSERT_NEAR(battery_level, 100.0f, 0.0000001);
}

TEST(bt_core_actions, patrol_btn)
{
  auto node = rclcpp::Node::make_shared("patrol_btn_node");
  auto node_sink = std::make_shared<bt_patrolling_tests::VelocitySinkNode>();

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("patrol_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Patrol    name="patrol"/>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;
    rclcpp::spin_some(node_sink->get_node_base_interface());
    rate.sleep();
  }

  ASSERT_FALSE(node_sink->vel_msgs_.empty());
  ASSERT_NEAR(node_sink->vel_msgs_.size(), 150, 2);

  geometry_msgs::msg::Twist & one_twist = node_sink->vel_msgs_.front();

  ASSERT_GT(one_twist.angular.z, 0.1);
  ASSERT_NEAR(one_twist.linear.x, 0.0, 0.0000001);
}

TEST(bt_core_actions, battery_checker_btn)
{
  auto node = rclcpp::Node::make_shared("battery_checker_btn_node");
  auto vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/output_vel", 100);

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("battery_checker_bt_node"));
  factory.registerFromPlugin(loader.getOSName("patrol_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <ReactiveSequence>
              <BatteryChecker    name="battery_checker"/>
              <Patrol    name="patrol"/>
          </ReactiveSequence>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  rclcpp::Rate rate(10);
  geometry_msgs::msg::Twist vel;
  vel.linear.x = 0.8;

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    vel_pub->publish(vel);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  float battery_level;
  ASSERT_TRUE(blackboard->get("battery_level", battery_level));
  ASSERT_NEAR(battery_level, 94.6, 1.0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
