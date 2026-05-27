#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuPrefilterNode : public rclcpp::Node {
public:
    ImuPrefilterNode() : Node("imu_prefilter_node") {
        this->declare_parameter("input_topic", "/livox/imu");
        this->declare_parameter("output_topic", "/imu_filtered");
        this->declare_parameter("filter_window_size", 5);

        input_topic_ = this->get_parameter("input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        window_size_ = this->get_parameter("filter_window_size").as_int();

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic_, 1000,
            std::bind(&ImuPrefilterNode::imu_callback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "IMU Prefilter Node started");
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto filtered_msg = *msg;
        imu_pub_->publish(filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::string input_topic_;
    std::string output_topic_;
    int window_size_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuPrefilterNode>());
    rclcpp::shutdown();
    return 0;
}
