#include "ActionNavigate.hpp"

ActionNavigate::ActionNavigate(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config), nav_finished_(false) {
    node_ = std::make_shared<rclcpp::Node>("action_navigate");
    goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
}

BT::NodeStatus ActionNavigate::onStart() {
    auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (goal) {
        goal_pub_->publish(goal.value());
        nav_finished_ = false;
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ActionNavigate::onRunning() {
    rclcpp::spin_some(node_);
    if (nav_finished_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void ActionNavigate::onHalted() {
}
