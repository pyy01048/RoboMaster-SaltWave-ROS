#include <rclcpp/rclcpp.hpp>
#include <sentinel_msgs/msg/sentinel_mode.hpp>
#include <sentinel_msgs/msg/armor_target.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class StateManagerNode : public rclcpp::Node {
public:
    StateManagerNode() : Node("state_manager_node") {
        mode_pub_ = this->create_publisher<sentinel_msgs::msg::SentinelMode>("/sentinel_mode", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        RCLCPP_INFO(this->get_logger(), "State Manager Node started");
    }

private:
    rclcpp::Publisher<sentinel_msgs::msg::SentinelMode>::SharedPtr mode_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateManagerNode>());
    rclcpp::shutdown();
    return 0;
}
