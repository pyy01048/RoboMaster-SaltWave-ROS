#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <sentinel_msgs/msg/armor_target.hpp>
#include <geometry_msgs/msg/twist.hpp>

class ConditionTargetVisible : public BT::ConditionNode {
public:
    ConditionTargetVisible(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sentinel_msgs::msg::ArmorTarget>::SharedPtr target_sub_;
    sentinel_msgs::msg::ArmorTarget::SharedPtr last_target_;
    bool target_visible_;
};
