#include "decision_making/bt_nodes.hpp"

namespace decision_making {

IsEnemyVisible::IsEnemyVisible(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), target_visible_(false) {
    node_ = std::make_shared<rclcpp::Node>("is_enemy_visible_bt");
    target_sub_ = node_->create_subscription<sentinel_msgs::msg::ArmorTarget>(
        "/armor_target", 10,
        [this](const sentinel_msgs::msg::ArmorTarget::SharedPtr msg) {
            target_visible_ = msg->visible;
        });
}

BT::NodeStatus IsEnemyVisible::tick() {
    rclcpp::spin_some(node_);
    return target_visible_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

PatrolAction::PatrolAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), patrol_active_(false) {
    node_ = std::make_shared<rclcpp::Node>("patrol_action_bt");
}

BT::NodeStatus PatrolAction::tick() {
    return BT::NodeStatus::SUCCESS;
}

TrackAction::TrackAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), tracking_active_(false) {
    node_ = std::make_shared<rclcpp::Node>("track_action_bt");
}

BT::NodeStatus TrackAction::tick() {
    return BT::NodeStatus::SUCCESS;
}

SetMode::SetMode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {
    node_ = std::make_shared<rclcpp::Node>("set_mode_bt");
}

BT::NodeStatus SetMode::tick() {
    return BT::NodeStatus::SUCCESS;
}

}
