#pragma once

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <atomic>

namespace decision_making {

class PatrolActionNode : public BT::StatefulActionNode {
public:
    PatrolActionNode(const std::string& name, const BT::NodeConfiguration& config);
    ~PatrolActionNode() = default;

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints"),
            BT::InputPort<bool>("loop", "true")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_client_;
    
    std::atomic<bool> goal_completed_;
    std::atomic<bool> goal_succeeded_;
    std::mutex mutex_;
    
    std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult> goal_future_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr goal_handle_;
    
    void goalResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr& goal_handle);
    void goalFeedbackCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::SharedPtr,
                              const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback);
    void goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>::WrappedResult& result);
};

class NavigateToPoseNode : public BT::StatefulActionNode {
public:
    NavigateToPoseNode(const std::string& name, const BT::NodeConfiguration& config);
    ~NavigateToPoseNode() = default;

    static BT::PortsList providedPorts() {
        return { 
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("theta", "0.0")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    
    std::atomic<bool> goal_completed_;
    std::atomic<bool> goal_succeeded_;
    std::mutex mutex_;
    
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
    
    void goalResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle);
    void goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);
};

class IsGoalReached : public BT::ConditionNode {
public:
    IsGoalReached(const std::string& name, const BT::NodeConfiguration& config);
    
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("tolerance", "0.2") };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose goal_pose_;
    bool goal_set_;
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

}
