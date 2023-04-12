#ifndef BT_PATROLLING__MOVE_HPP_
#define BT_PATROLLING__MOVE_HPP_

#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "bt_patrolling/ctrl_support/BTActionNode.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace bt_patrolling
{
class Move : public bt_patrolling::BtActionNode<nav2_msgs::action::NavigateToPose>
{
public:
    explicit Move(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;
    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return{
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
        };
    }
};

}
#endif