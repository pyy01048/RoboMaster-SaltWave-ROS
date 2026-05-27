#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <sentinel_msgs/msg/sentinel_mode.hpp>

class ActionSetMode : public BT::SyncActionNode {
public:
    ActionSetMode(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<uint8_t>("mode", "PATROL", "The mode to set")
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sentinel_msgs::msg::SentinelMode>::SharedPtr mode_pub_;
};
