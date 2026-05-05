#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"

#include "bt_patrolling/TrackObjects.hpp"
#include "bt_test_utils.hpp"

using namespace std::chrono_literals;

TEST(tracking_lifecycle, track_objects_btn_1)
{
  auto node = rclcpp::Node::make_shared("track_objects_btn_node");
  auto node_head_tracker = rclcpp_lifecycle::LifecycleNode::make_shared("head_tracker");

  std::atomic_bool finish{false};
  std::thread t([&]() {
      while (!finish.load()) {rclcpp::spin_some(node_head_tracker->get_node_base_interface());}
    });

  BT::NodeConfiguration conf;
  conf.blackboard = BT::Blackboard::create();
  conf.blackboard->set("node", node);
  bt_patrolling::BtLifecycleCtrlNode bt_node("TrackObjects", "head_tracker", conf);

  bt_node.change_state_client_ = bt_node.createServiceClient<lifecycle_msgs::srv::ChangeState>(
    "/head_tracker/change_state");
  ASSERT_TRUE(bt_node.change_state_client_->service_is_ready());

  bt_node.get_state_client_ = bt_node.createServiceClient<lifecycle_msgs::srv::GetState>(
    "/head_tracker/get_state");
  ASSERT_TRUE(bt_node.get_state_client_->service_is_ready());
  auto start = node->now();

  rclcpp::Rate rate(10);
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_EQ(bt_node.get_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  bt_node.ctrl_node_state_ = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  ASSERT_FALSE(bt_node.set_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));

  node_head_tracker->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  bt_node.ctrl_node_state_ = bt_node.get_state();

  ASSERT_TRUE(bt_node.set_state(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE));
  ASSERT_EQ(bt_node.get_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  bt_node.ctrl_node_state_ = bt_node.get_state();

  ASSERT_TRUE(bt_node.set_state(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE));
  ASSERT_EQ(bt_node.get_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish.store(true);
  t.join();
}

TEST(tracking_lifecycle, track_objects_btn_2)
{
  auto node = rclcpp::Node::make_shared("track_objects_btn_node");
  auto node_head_tracker = rclcpp_lifecycle::LifecycleNode::make_shared("head_tracker");

  std::atomic_bool finish{false};
  std::thread t([&]() {
      while (!finish.load()) {rclcpp::spin_some(node_head_tracker->get_node_base_interface());}
    });

  BT::NodeConfiguration conf;
  conf.blackboard = BT::Blackboard::create();
  conf.blackboard->set("node", node);
  bt_patrolling::BtLifecycleCtrlNode bt_node("TrackObjects", "head_tracker", conf);

  node_head_tracker->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::Rate rate(10);
  auto start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_EQ(bt_node.tick(), BT::NodeStatus::RUNNING);

  ASSERT_TRUE(bt_node.change_state_client_->service_is_ready());
  ASSERT_TRUE(bt_node.get_state_client_->service_is_ready());

  ASSERT_EQ(bt_node.get_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  ASSERT_EQ(bt_node.tick(), BT::NodeStatus::RUNNING);

  bt_node.halt();

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_EQ(bt_node.get_state(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish.store(true);
  t.join();
}

TEST(tracking_lifecycle, track_objects_btn_3)
{
  auto node = rclcpp::Node::make_shared("track_objects_btn_node");
  auto node_head_tracker = rclcpp_lifecycle::LifecycleNode::make_shared("head_tracker");

  node_head_tracker->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  std::atomic_bool finish{false};
  std::thread t([&]() {
      while (!finish.load()) {rclcpp::spin_some(node_head_tracker->get_node_base_interface());}
    });

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("track_objects_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <KeepRunningUntilFailure>
              <TrackObjects    name="track_objects"/>
          </KeepRunningUntilFailure>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);
  auto start = node->now();
  rclcpp::Rate rate(10);

  {
    BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

    ASSERT_EQ(
      node_head_tracker->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    while (rclcpp::ok() && (node->now() - start) < 1s) {
      tree.rootNode()->executeTick() == BT::NodeStatus::RUNNING;

      rclcpp::spin_some(node);
      rate.sleep();
    }
    ASSERT_EQ(
      node_head_tracker->get_current_state().id(),
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  }

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  ASSERT_EQ(
    node_head_tracker->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish.store(true);
  t.join();
}

TEST(tracking_lifecycle, move_track_btn)
{
  auto node = rclcpp::Node::make_shared("move_btn_node");
  auto nav2_fake_node = std::make_shared<bt_patrolling_tests::Nav2FakeServer>();
  auto node_head_tracker = rclcpp_lifecycle::LifecycleNode::make_shared("head_tracker");

  node_head_tracker->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  nav2_fake_node->start_server();

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(nav2_fake_node);
  exe.add_node(node_head_tracker->get_node_base_interface());
  std::atomic_bool finish{false};
  std::thread t([&]() {
      while (!finish.load()) {exe.spin_some();}
    });

  BT::BehaviorTreeFactory factory;
  BT::SharedLibrary loader;

  factory.registerFromPlugin(loader.getOSName("move_bt_node"));
  factory.registerFromPlugin(loader.getOSName("track_objects_bt_node"));

  std::string xml_bt =
    R"(
    <root main_tree_to_execute = "MainTree" >
      <BehaviorTree ID="MainTree">
          <Parallel success_threshold="1" failure_threshold="1">
            <TrackObjects    name="track_objects"/>
            <Move    name="move" goal="{goal}"/>
          </Parallel>
      </BehaviorTree>
    </root>)";

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  geometry_msgs::msg::PoseStamped goal;
  blackboard->set("goal", goal);

  BT::Tree tree = factory.createTreeFromText(xml_bt, blackboard);

  EXPECT_EQ(
    node_head_tracker->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  rclcpp::Rate rate(10);
  auto start = node->now();
  auto finish_tree = false;
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    finish_tree = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  EXPECT_FALSE(finish_tree);
  EXPECT_EQ(
    node_head_tracker->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  while (rclcpp::ok() && !finish_tree) {
    finish_tree = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  start = node->now();
  while (rclcpp::ok() && (node->now() - start) < 1s) {
    rclcpp::spin_some(node);
    rate.sleep();
  }

  EXPECT_EQ(
    node_head_tracker->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  finish.store(true);
  t.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
