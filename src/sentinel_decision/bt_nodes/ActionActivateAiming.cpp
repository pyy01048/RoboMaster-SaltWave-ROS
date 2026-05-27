#include "ActionActivateAiming.hpp"

ActionActivateAiming::ActionActivateAiming(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config), aiming_activated_(false) {
    node_ = std::make_shared<rclcpp::Node>("action_activate_aiming");
}

BT::NodeStatus ActionActivateAiming::tick() {
    aiming_activated_ = true;
    return BT::NodeStatus::SUCCESS;
}
