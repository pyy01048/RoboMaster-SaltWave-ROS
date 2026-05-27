#include "ConditionTargetVisible.hpp"

ConditionTargetVisible::ConditionTargetVisible(const std::string& name, const BT::NodeConfiguration& config)
  : BT::ConditionNode(name, config), target_visible_(false) {
    node_ = std::make_shared<rclcpp::Node>("condition_target_visible");
    target_sub_ = node_->create_subscription<sentinel_msgs::msg::ArmorTarget>(
        "/armor_target", 10,
        [this](const sentinel_msgs::msg::ArmorTarget::SharedPtr msg) {
            last_target_ = msg;
            target_visible_ = msg->visible;
        });
}

BT::NodeStatus ConditionTargetVisible::tick() {
    rclcpp::spin_some(node_);
    if (target_visible_) {
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}
