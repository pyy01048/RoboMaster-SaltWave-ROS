#include <rclcpp/rclcpp.hpp>
#include <sentinel_msgs/msg/aim_target.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class ServoControllerNode : public rclcpp::Node {
public:
    ServoControllerNode() : Node("servo_controller_node") {
        this->declare_parameter("aim_target_topic", "/aim_target");
        this->declare_parameter("gimbal_cmd_topic", "/gimbal_cmd");
        this->declare_parameter("yaw_p", 1.0);
        this->declare_parameter("yaw_i", 0.0);
        this->declare_parameter("yaw_d", 0.0);
        this->declare_parameter("pitch_p", 1.0);
        this->declare_parameter("pitch_i", 0.0);
        this->declare_parameter("pitch_d", 0.0);

        aim_target_topic_ = this->get_parameter("aim_target_topic").as_string();
        gimbal_cmd_topic_ = this->get_parameter("gimbal_cmd_topic").as_string();
        yaw_p_ = this->get_parameter("yaw_p").as_double();
        yaw_i_ = this->get_parameter("yaw_i").as_double();
        yaw_d_ = this->get_parameter("yaw_d").as_double();
        pitch_p_ = this->get_parameter("pitch_p").as_double();
        pitch_i_ = this->get_parameter("pitch_i").as_double();
        pitch_d_ = this->get_parameter("pitch_d").as_double();

        aim_target_sub_ = this->create_subscription<sentinel_msgs::msg::AimTarget>(
            aim_target_topic_, 10,
            std::bind(&ServoControllerNode::target_callback, this, std::placeholders::_1));

        gimbal_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(gimbal_cmd_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Servo Controller Node started");
    }

private:
    void target_callback(const sentinel_msgs::msg::AimTarget::SharedPtr msg) {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header = msg->header;

        gimbal_cmd_pub_->publish(cmd);
    }

    rclcpp::Subscription<sentinel_msgs::msg::AimTarget>::SharedPtr aim_target_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr gimbal_cmd_pub_;

    std::string aim_target_topic_;
    std::string gimbal_cmd_topic_;
    double yaw_p_;
    double yaw_i_;
    double yaw_d_;
    double pitch_p_;
    double pitch_i_;
    double pitch_d_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoControllerNode>());
    rclcpp::shutdown();
    return 0;
}
