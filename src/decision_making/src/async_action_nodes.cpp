#include "decision_making/async_action_nodes.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace decision_making {

PatrolActionNode::PatrolActionNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
    , goal_completed_(false)
    , goal_succeeded_(false) {
    node_ = std::make_shared<rclcpp::Node>("patrol_action_bt_node");
    waypoint_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(node_, "follow_waypoints");
}

BT::NodeStatus PatrolActionNode::onStart() {
    auto waypoints_input = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints");
    if (!waypoints_input) {
        return BT::NodeStatus::FAILURE;
    }
    
    auto waypoints = waypoints_input.value();
    if (waypoints.empty()) {
        return BT::NodeStatus::FAILURE;
    }
    
    if (!waypoint_client_->wait_for_action_server(std::chrono::seconds(5))) {
        return BT::NodeStatus::FAILURE;
    }
    
    goal_completed_ = false;
    goal_succeeded_ = false;
    
    auto goal_msg = nav2_msgs::action::FollowWaypoints::Goal();
    goal_msg.poses = waypoints;
    
    auto goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    goal_options.goal_response_callback = 
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr& goal_handle) {
            this->goalResponseCallback(goal_handle);
        };
    goal_options.feedback_callback = 
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr goal_handle,
               const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback) {
            this->goalFeedbackCallback(goal_handle, feedback);
        };
    goal_options.result_callback = 
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult& result) {
            this->goalResultCallback(result);
        };
    
    waypoint_client_->async_send_goal(goal_msg, goal_options);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PatrolActionNode::onRunning() {
    rclcpp::spin_some(node_);
    
    if (goal_completed_) {
        return goal_succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::RUNNING;
}

void PatrolActionNode::onHalted() {
    if (goal_handle_) {
        waypoint_client_->async_cancel_goal(goal_handle_);
    }
}

void PatrolActionNode::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr& goal_handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_ = goal_handle;
}

void PatrolActionNode::goalFeedbackCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback) {
}

void PatrolActionNode::goalResultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_completed_ = true;
    goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
}

NavigateToPoseNode::NavigateToPoseNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
    , goal_completed_(false)
    , goal_succeeded_(false) {
    node_ = std::make_shared<rclcpp::Node>("navigate_to_pose_bt_node");
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
}

BT::NodeStatus NavigateToPoseNode::onStart() {
    auto x_input = getInput<double>("x");
    auto y_input = getInput<double>("y");
    auto theta_input = getInput<double>("theta");
    
    if (!x_input || !y_input) {
        return BT::NodeStatus::FAILURE;
    }
    
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
        return BT::NodeStatus::FAILURE;
    }
    
    goal_completed_ = false;
    goal_succeeded_ = false;
    
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = node_->now();
    goal_msg.pose.pose.position.x = x_input.value();
    goal_msg.pose.pose.position.y = y_input.value();
    goal_msg.pose.pose.position.z = 0.0;
    
    double theta = theta_input.value_or(0.0);
    goal_msg.pose.pose.orientation.w = std::cos(theta / 2.0);
    goal_msg.pose.pose.orientation.z = std::sin(theta / 2.0);
    
    auto goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    goal_options.goal_response_callback = 
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle) {
            this->goalResponseCallback(goal_handle);
        };
    goal_options.result_callback = 
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
            this->goalResultCallback(result);
        };
    
    nav_client_->async_send_goal(goal_msg, goal_options);
    
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseNode::onRunning() {
    rclcpp::spin_some(node_);
    
    if (goal_completed_) {
        return goal_succeeded_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::RUNNING;
}

void NavigateToPoseNode::onHalted() {
    if (goal_handle_) {
        nav_client_->async_cancel_goal(goal_handle_);
    }
}

void NavigateToPoseNode::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle) {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_handle_ = goal_handle;
}

void NavigateToPoseNode::goalResultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_completed_ = true;
    goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
}

IsGoalReached::IsGoalReached(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
    , goal_set_(false) {
    node_ = std::make_shared<rclcpp::Node>("is_goal_reached_bt_node");
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom_filtered", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            this->odomCallback(msg);
        });
}

BT::NodeStatus IsGoalReached::tick() {
    rclcpp::spin_some(node_);
    
    auto tolerance_input = getInput<double>("tolerance");
    double tolerance = tolerance_input.value_or(0.2);
    
    if (!goal_set_) {
        return BT::NodeStatus::FAILURE;
    }
    
    double dx = current_pose_.position.x - goal_pose_.position.x;
    double dy = current_pose_.position.y - goal_pose_.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    
    return (dist < tolerance) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void IsGoalReached::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
}

}
