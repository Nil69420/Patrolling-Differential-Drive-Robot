#include <string>
#include <iostream>
#include "bt_patrolling/GetWaypoint.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace bt_patrolling
{
int GetWaypoint::current_ = 0;

GetWaypoint::GetWaypoint(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
 :  BT::ActionNodeBase(xml_tag_name, conf)
{
    rclcpp::Node::SharedPtr node;
    config().blackboard->get("node", node);

    geometry_msgs::msg::PoseStamped wp;
    wp.header.frame_id = "map";
    wp.pose.orientation.w = 1.0;

    wp.pose.position.x = 3.67;
    wp.pose.position.y = -0.24;
    recharge_point_ = wp;

    // wp1
    wp.pose.position.x = 1.07;
    wp.pose.position.y = -12.38;
    waypoints_.push_back(wp);

    wp.pose.position.x = -5.52;
    wp.pose.position.y = -8.85;
    waypoints_.push_back(wop);

    wp.pose.position.x = -0.56;
    wp.pose.position.y = 0.24;
    waypoints_.push_back(wp);
}

void
GetWaypoint::halt()
{
}

BT::NodeStatus
GetWaypoint::tick()
{
    std::string_id;
    getInput("wp_id", id);

    if (id == "recharge")
    {
        setOutput("waypoint", recharge_point_);
    }
    else
    {
        setOutput("waypoint", waypoints_[current_++]);
        current_ = current_ % waypoints_.size();
    }

    return BT::NodeStatus::SUCCESS;
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_patrolling::GetWaypoint>("GetWaypoint");
}