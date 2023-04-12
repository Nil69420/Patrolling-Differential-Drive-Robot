#ifndef BT_PATROLLING__RECHARGE_HPP_
#define BT_PATROLLING__RECHARGE_HPP_

#include <string>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace bt_patrolling
{
class Recharge : public BT::ActionNodeBase
{
public:
    explicit Recharge(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf);
    
    void halt();
    
    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return BT::PortsList({});
    }
private:
    int counter_;
};
}
#endif