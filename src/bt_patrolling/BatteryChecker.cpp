#include <string>
#include <iostream>
#include <algorithm>
#include "bt_patrolling/BatteryChecker.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_patrolling
{
using namespace std::chrono_literals;
using namespace std::placeholders;

BatteryChecker::BatteryChecker(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
:  BT::ConditionNode(xml_tag_name, conf)
{
    config().blackboard->get("node", node_);

    vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/output_vel", 100, std::bind(&BatteryChecker::vel_callback, this, _1));
    
    last_reading_time_ = node_->now();
}

void
BatteryChecker::vel_callback(const geometry_msgs::Twist::SharedPtr msg)
{
    last_twist_ = *msg;
}

void
BatteryChecker::update_battery()
{
    float battery_level;
    if(!config().blackboard->get("battery_level", battery_level))
    {
        battery_level = 100.0f;
    }

    float dt = (node_->now() - last_reading_time_).seconds();
    last_reading_time_ = node_->now();

    float vel = sqrt(last_twist.linear.x * last_twist_.linear.x + last_twist_.angular.z * last_twist_.angular.z);
    battery_level = std::max(0.f, battery_level - (vel * dt * DECAY_LEVEL) - EPSILON *dt);

    config().blackboard->set("battery_level", battery_level);
}

BT::NodeStatus
BatteryChecker::tick()
{
    update_battery();
    float battery_level;
    config().blackboard->get("battery_level", battery_level);

    std::cout << battery_level << std::endl;

    if (battery_level < MIN_LEVEL)
    {
        return BT::NodeStatus::FAILURE;
    }
    else
    {
        return BT::NodeStatus::SUCCESS;
    }
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_patrolling::BatteryChecker>("BatteryChecker");
}