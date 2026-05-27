#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>

class ActionActivateAiming : public BT::SyncActionNode {
public:
    ActionActivateAiming(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }

private:
    rclcpp::Node::SharedPtr node_;
    bool aiming_activated_;
};
