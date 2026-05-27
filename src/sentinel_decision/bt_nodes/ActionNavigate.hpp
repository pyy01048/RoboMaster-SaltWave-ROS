#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class ActionNavigate : public BT::StatefulActionNode {
public:
    ActionNavigate(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    bool nav_finished_;
};
