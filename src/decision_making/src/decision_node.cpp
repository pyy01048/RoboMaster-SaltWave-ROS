#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "decision_making/bt_nodes.hpp"

using namespace decision_making;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("decision_node");
    
    node->declare_parameter("tree_file", "");
    std::string tree_file = node->get_parameter("tree_file").as_string();
    
    if (tree_file.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Behavior tree file not specified!");
        rclcpp::shutdown();
        return 1;
    }
    
    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<IsEnemyVisible>("IsEnemyVisible");
    factory.registerNodeType<PatrolAction>("PatrolAction");
    factory.registerNodeType<TrackAction>("TrackAction");
    factory.registerNodeType<SetMode>("SetMode");
    
    auto tree = factory.createTreeFromFile(tree_file);
    
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        tree.tickRoot();
        rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}
