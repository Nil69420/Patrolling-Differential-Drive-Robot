#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include "bt_patrolling/TrackObjects.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace bt_patrolling
{
TrackObjects::TrackObjects(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf)
:   bt_patrolling::BtLifecycleCtrlNode(xml_tag_name, action_name, conf)
{
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builer =
        [](const std::string & name, const BT::NodeConfiguration & config)
        {
            return std::make_unique<bt_patrolling::TrackObjects>(
                name, "/head_tracker", config);
            
        };

        factory.registerBuilder<bt_patrolling::TrackObjects>(
            "TrackObjects", builder);
        
}