#include <rclcpp/rclcpp.hpp>
#include <sentinel_msgs/msg/tracked_target.hpp>
#include <sentinel_msgs/msg/aim_target.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class BallisticsNode : public rclcpp::Node {
public:
    BallisticsNode() : Node("ballistics_node") {
        this->declare_parameter("tracked_targets_topic", "/tracked_targets");
        this->declare_parameter("aim_target_topic", "/aim_target");
        this->declare_parameter("bullet_speed", 25.0);

        tracked_targets_topic_ = this->get_parameter("tracked_targets_topic").as_string();
        aim_target_topic_ = this->get_parameter("aim_target_topic").as_string();
        bullet_speed_ = this->get_parameter("bullet_speed").as_double();

        tracked_targets_sub_ = this->create_subscription<sentinel_msgs::msg::TrackedTarget>(
            tracked_targets_topic_, 10,
            std::bind(&BallisticsNode::target_callback, this, std::placeholders::_1));

        aim_target_pub_ = this->create_publisher<sentinel_msgs::msg::AimTarget>(aim_target_topic_, 10);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Ballistics Node started");
    }

private:
    void target_callback(const sentinel_msgs::msg::TrackedTarget::SharedPtr msg) {
        sentinel_msgs::msg::AimTarget aim_target;
        aim_target.header = msg->header;
        aim_target.target_id = msg->track_id;
        aim_target.confidence = msg->confidence;
        aim_target.point = msg->pose.position;

        aim_target_pub_->publish(aim_target);
    }

    rclcpp::Subscription<sentinel_msgs::msg::TrackedTarget>::SharedPtr tracked_targets_sub_;
    rclcpp::Publisher<sentinel_msgs::msg::AimTarget>::SharedPtr aim_target_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string tracked_targets_topic_;
    std::string aim_target_topic_;
    double bullet_speed_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BallisticsNode>());
    rclcpp::shutdown();
    return 0;
}
