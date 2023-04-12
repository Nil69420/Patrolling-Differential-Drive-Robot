#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include "bt_patrolling/Move.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace bt_patrolling
{
Move::Move(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
 :  bt_patrolling::BtActionNode<nav2_msgs::action::NavigateToPose>(xml_tag_name, action_name, conf)
{
}

void
Move::on_tick()
{
    geometry_msgs::msg::PoseStamped goal;
    getInput("goal", goal);

    goal_.pose = goal;
}

BT::NodeStatus
Move::on_success()
{
    RCLCPP_INFO(node->get_logger(), "navigation success");

    return BT::NodeStatus::SUCCESS;
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
        [](const std::string & name, const BT::NodeConfiguration & config)
        {
            return std::make_unique<bt_patrolling::Move>(
                name, "navigate_to_pose", config);
            
        };
    
    factory.registerBuilder<bt_patrolling::Move>(
        "Move", builder);
    
}