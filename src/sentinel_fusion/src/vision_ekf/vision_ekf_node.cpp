#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <deque>
#include <mutex>
#include <Eigen/Dense>

class VisionEKFNode : public rclcpp::Node {
public:
    VisionEKFNode() : Node("vision_ekf_node") {
        this->declare_parameter("imu_topic", "/imu_filtered");
        this->declare_parameter("odom_topic", "/Odometry");
        this->declare_parameter("odom_pub_topic", "/odom_vision");
        this->declare_parameter("vision_update_rate", 60.0);
        this->declare_parameter("vision_process_noise_std", 0.01);
        this->declare_parameter("vision_measurement_noise_std", 0.1);

        imu_topic_ = this->get_parameter("imu_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        odom_pub_topic_ = this->get_parameter("odom_pub_topic").as_string();
        vision_update_rate_ = this->get_parameter("vision_update_rate").as_double();
        vision_process_noise_std_ = this->get_parameter("vision_process_noise_std").as_double();
        vision_measurement_noise_std_ = this->get_parameter("vision_measurement_noise_std").as_double();

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 1000,
            std::bind(&VisionEKFNode::imu_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&VisionEKFNode::odom_callback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_pub_topic_, 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        init_ekf();
        update_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / vision_update_rate_),
            std::bind(&VisionEKFNode::update_callback, this));

        RCLCPP_INFO(this->get_logger(), "Vision EKF Node started at %.2f Hz", vision_update_rate_);
    }

private:
    void init_ekf() {
        x_.setZero();
        P_.setIdentity();
        P_ *= 0.01;
        Q_.setIdentity();
        Q_ *= vision_process_noise_std_ * vision_process_noise_std_;
        R_.setIdentity();
        R_ *= vision_measurement_noise_std_ * vision_measurement_noise_std_;
        last_time_ = this->now();
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!first_imu_) {
            first_imu_ = true;
            last_time_ = msg->header.stamp;
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_odom_ = msg;
    }

    void update_callback() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!last_odom_) return;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = x_(0);
        odom_msg.pose.pose.position.y = x_(1);
        odom_msg.pose.pose.position.z = x_(2);
        odom_msg.pose.pose.orientation.w = x_(3);
        odom_msg.pose.pose.orientation.x = x_(4);
        odom_msg.pose.pose.orientation.y = x_(5);
        odom_msg.pose.pose.orientation.z = x_(6);

        odom_pub_->publish(odom_msg);
    }

    Eigen::Matrix<double, 7, 1> x_;
    Eigen::Matrix<double, 7, 7> P_, Q_, R_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    nav_msgs::msg::Odometry::SharedPtr last_odom_;
    std::mutex mutex_;
    bool first_imu_ = false;
    rclcpp::Time last_time_;

    std::string imu_topic_;
    std::string odom_topic_;
    std::string odom_pub_topic_;
    double vision_update_rate_;
    double vision_process_noise_std_;
    double vision_measurement_noise_std_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionEKFNode>());
    rclcpp::shutdown();
    return 0;
}
