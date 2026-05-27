#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

class PointLIOWrapperNode : public rclcpp::Node {
public:
    PointLIOWrapperNode() : Node("point_lio_wrapper_node") {
        this->declare_parameter("pointcloud_topic", "/livox/lidar");
        this->declare_parameter("imu_topic", "/imu_filtered");
        this->declare_parameter("odom_topic", "/Odometry");

        pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 10,
            std::bind(&PointLIOWrapperNode::pointcloud_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 1000,
            std::bind(&PointLIOWrapperNode::imu_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

        RCLCPP_INFO(this->get_logger(), "Point-LIO Wrapper Node started");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pointcloud_pub_->publish(*msg);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        imu_pub_->publish(*msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    std::string pointcloud_topic_;
    std::string imu_topic_;
    std::string odom_topic_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointLIOWrapperNode>());
    rclcpp::shutdown();
    return 0;
}
