#ifndef BT_PATROLLING__BATTERYCHECKER_HPP_
#define BT_PATROLLING__BATTERYCHECKER_HPP_

#include <string>
#include <vector>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_patrolling
{
class BatteryChecker : public BT::ConditionNode
{
public:
    explicit BatteryChecker(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf);
    
    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return BT::PortsList({});
    }

    void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

    const float DECAY_LEVEL = 0.5;
    const float EPSILON = 0.01;
    const float MIN_LEVEL = 10.0;

private:
    void update_battery();
    rclcpp::Node::SharedPtr node_;
    rclcpp::Time last_reading_time_;
    geometry_msgs::msg::Twist last_twist_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
};
}

#endif