#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "multi_rate_ekf.hpp"

namespace sensor_fusion {

class FusionNode : public rclcpp::Node {
public:
    FusionNode();
    ~FusionNode() = default;

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishFusedState();
    void loadParameters();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<MultiRateEKF> ekf_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::deque<sensor_fusion::ImuData> imu_buffer_;
    double last_imu_time_;
    double last_lidar_time_;

    double imu_process_noise_;
    double lidar_meas_noise_;
    int publish_rate_;
};

}
