#include "sensor_fusion/fusion_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sensor_fusion {

FusionNode::FusionNode() : Node("fusion_node"), last_imu_time_(0.0), last_lidar_time_(0.0) {
    loadParameters();

    ekf_ = std::make_unique<MultiRateEKF>();

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", rclcpp::SensorDataQoS(),
        std::bind(&FusionNode::imuCallback, this, std::placeholders::_1));

    lidar_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastlio/odom", rclcpp::SensorDataQoS(),
        std::bind(&FusionNode::lidarOdomCallback, this, std::placeholders::_1));

    fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom_filtered", rclcpp::SensorDataQoS());

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / publish_rate_),
        std::bind(&FusionNode::publishFusedState, this));

    RCLCPP_INFO(this->get_logger(), "Sensor Fusion Node initialized with %d Hz publish rate", publish_rate_);
}

void FusionNode::loadParameters() {
    this->declare_parameter("imu_process_noise", 0.01);
    this->declare_parameter("lidar_meas_noise", 0.05);
    this->declare_parameter("publish_rate", 200);

    imu_process_noise_ = this->get_parameter("imu_process_noise").as_double();
    lidar_meas_noise_ = this->get_parameter("lidar_meas_noise").as_double();
    publish_rate_ = this->get_parameter("publish_rate").as_int();
}

void FusionNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    ImuData imu_data;
    imu_data.timestamp = timestamp;
    imu_data.accel = Eigen::Vector3d(msg->linear_acceleration.x, 
                                     msg->linear_acceleration.y, 
                                     msg->linear_acceleration.z);
    imu_data.gyro = Eigen::Vector3d(msg->angular_velocity.x, 
                                    msg->angular_velocity.y, 
                                    msg->angular_velocity.z);

    if (last_imu_time_ > 0.0) {
        double dt = timestamp - last_imu_time_;
        if (dt > 0.0 && dt < 0.1) {
            ekf_->predict(imu_data, dt);
        }
    }

    last_imu_time_ = timestamp;
    imu_buffer_.push_back(imu_data);
    if (imu_buffer_.size() > 1000) {
        imu_buffer_.pop_front();
    }
}

void FusionNode::lidarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    LidarOdomData lidar_data;
    lidar_data.timestamp = timestamp;
    lidar_data.position = Eigen::Vector3d(msg->pose.pose.position.x,
                                          msg->pose.pose.position.y,
                                          msg->pose.pose.position.z);
    lidar_data.orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                msg->pose.pose.orientation.x,
                                                msg->pose.pose.orientation.y,
                                                msg->pose.pose.orientation.z);
    lidar_data.linear_velocity = Eigen::Vector3d(msg->twist.twist.linear.x,
                                                  msg->twist.twist.linear.y,
                                                  msg->twist.twist.linear.z);

    ekf_->fuseLidar(lidar_data);
    last_lidar_time_ = timestamp;
}

void FusionNode::publishFusedState() {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    Eigen::Vector3d pos = ekf_->getPosition();
    Eigen::Quaterniond orient = ekf_->getOrientation();
    Eigen::Vector3d vel = ekf_->getVelocity();

    odom_msg.pose.pose.position.x = pos.x();
    odom_msg.pose.pose.position.y = pos.y();
    odom_msg.pose.pose.position.z = pos.z();

    odom_msg.pose.pose.orientation.w = orient.w();
    odom_msg.pose.pose.orientation.x = orient.x();
    odom_msg.pose.pose.orientation.y = orient.y();
    odom_msg.pose.pose.orientation.z = orient.z();

    odom_msg.twist.twist.linear.x = vel.x();
    odom_msg.twist.twist.linear.y = vel.y();
    odom_msg.twist.twist.linear.z = vel.z();

    fused_odom_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = odom_msg.header.stamp;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = pos.x();
    transform.transform.translation.y = pos.y();
    transform.transform.translation.z = pos.z();
    transform.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
}

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensor_fusion::FusionNode>());
    rclcpp::shutdown();
    return 0;
}
