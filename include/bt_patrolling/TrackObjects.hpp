#ifndef BT_PATROLLING__TRACKOBJECTS_HPP_
#define BT_PATROLLING__TRACKOBJECTS_HPP_

#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "bt_patrolling/ctrl_support/BTLifeCycleCtrlNode.hpp"

namespace bt_patrolling
{
class TrackObjects : public bt_patrolling::BtLifecycleCtrlNode
{
public:
    explicit TrackObjects(
        const std::string & xml_tag_name,
        const std::string & node_name,
        const BT::NodeConfiguration & conf);
    

    static BT::PortsList providedPorts()
    {
        return BT::PortsList({});
    }
};

}
#endif