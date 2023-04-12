#ifndef BT_PATROLLING__PATROL_HPP_
#define BT_PATROLLING__PATROL_HPP_

#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace bt_patrolling
{
class Patrol : public BT::ActionNodeBase
{
public:
    explicit Patrol(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf);
    
    void halt();
    
    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return BT::PortsList({});
    }
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Time start_time_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};
}
#endif