#include <string>
#include <iostream>
#include <set>
#include "bt_patrolling/Recharge.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace bt_patrolling
{
Recharge::Recharge(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
 :  BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void 
Recharge::halt()
{
}

BT::NodeStatus
Recharge::tick()
{
    std::cout << "Recharge" << counter_ < std::endl;

    if(counter_++ < 50)
    {
        return BT::NodeStatus::RUNNING;
    }
    else
    {
    counter_ = 0;
    config().blackboard->set<float>("battery_level", 100.0f);
    return BT::NodeStatus::SUCCESS;
    }
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<bt_patrolling::Recharge>("Recharge");
}