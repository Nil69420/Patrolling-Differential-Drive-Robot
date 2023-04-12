#include <string>
#include <memory>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("patrolling_node");

    BT::BehaviorTreeFactory factory;
    BT::SharedLibrary loader;

    factory.registerFromPlugin(loader.getOSName("battery_checker_bt_node"));
    factory.registerFromPlugin(loader.getOSName("patrol_bt_node"));
    factory.registerFromPlugin(loader.getOSName("recharge_bt_node"));
    factory.registerFromPlugin(loader.getOSName("move_bt_node"));
    factory.registerFromPlugin(loader.getOSName("get_waypoint_bt_node"));
    factory.registerFromPlugin(loader.getOSName("track_objects_bt_node"));

    std::string pkgpath = ament_index_cpp::get_package_share_directory("bt_patrolling");
    std::string xml_file = pkgpath + "/behavior_tree_xml/patrolling.xml";

    auto blackboard = BT::Blackboard::create();
    blackboard->set("node", node);
    BT::Tree tree = factory.createTreeFromFile(xml_file, blackboard);

    auto publisher_zmq = std::make_shared<BT::PublisherZMQ>(tree, 10, 2666, 2667);
    rclcpp::Rate rate(10);

    bool finish = false;
    while (!finish && rclcpp::ok())
    {
        finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}