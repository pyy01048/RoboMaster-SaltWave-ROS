#include "ActionSetMode.hpp"

ActionSetMode::ActionSetMode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {
    node_ = std::make_shared<rclcpp::Node>("action_set_mode");
    mode_pub_ = node_->create_publisher<sentinel_msgs::msg::SentinelMode>("/sentinel_mode", 10);
}

BT::NodeStatus ActionSetMode::tick() {
    auto mode = getInput<uint8_t>("mode").value();

    sentinel_msgs::msg::SentinelMode mode_msg;
    mode_msg.mode = mode;
    mode_pub_->publish(mode_msg);

    return BT::NodeStatus::SUCCESS;
}
