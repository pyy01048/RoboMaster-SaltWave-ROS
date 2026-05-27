#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sentinel_msgs/msg/armor_target.hpp>

namespace decision_making {

class IsEnemyVisible : public BT::ConditionNode {
public:
    IsEnemyVisible(const std::string& name, const BT::NodeConfiguration& config);
    
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sentinel_msgs::msg::ArmorTarget>::SharedPtr target_sub_;
    bool target_visible_;
};

class PatrolAction : public BT::SyncActionNode {
public:
    PatrolAction(const std::string& name, const BT::NodeConfiguration& config);
    
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }

private:
    rclcpp::Node::SharedPtr node_;
    bool patrol_active_;
};

class TrackAction : public BT::SyncActionNode {
public:
    TrackAction(const std::string& name, const BT::NodeConfiguration& config);
    
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() { return {}; }

private:
    rclcpp::Node::SharedPtr node_;
    bool tracking_active_;
};

class SetMode : public BT::SyncActionNode {
public:
    SetMode(const std::string& name, const BT::NodeConfiguration& config);
    
    BT::NodeStatus tick() override;
    
    static BT::PortsList providedPorts() {
        return { BT::InputPort<uint8_t>("mode") };
    }

private:
    rclcpp::Node::SharedPtr node_;
};

}
