#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "sensor_fusion/multi_rate_ekf_2d.hpp"
#include "sensor_fusion/imu_prefilter.hpp"

using namespace std::chrono_literals;

class SensorFusionNode : public rclcpp::Node {
public:
    SensorFusionNode() : Node("sensor_fusion_node") {
        this->declare_parameter("use_sim_time", false);
        this->declare_parameter("publish_tf", true);
        this->declare_parameter("publish_rate", 200);
        
        this->declare_parameter("ekf.process_noise_pos", 0.01);
        this->declare_parameter("ekf.process_noise_theta", 0.001);
        this->declare_parameter("ekf.process_noise_vel", 0.1);
        this->declare_parameter("ekf.process_noise_omega", 0.01);
        
        this->declare_parameter("ekf.lidar_noise_pos", 0.05);
        this->declare_parameter("ekf.lidar_noise_theta", 0.01);
        this->declare_parameter("ekf.wheel_noise_vel", 0.1);
        
        this->declare_parameter("ekf.max_lidar_age", 0.2);
        this->declare_parameter("ekf.max_imu_age", 0.1);
        this->declare_parameter("ekf.max_vision_age", 0.2);
        this->declare_parameter("ekf.max_encoder_age", 0.1);
        
        this->declare_parameter("imu.complementary_gain", 0.95);
        this->declare_parameter("imu.update_rate", 500.0);
        this->declare_parameter("imu.accel_magnitude_tolerance", 2.0);
        
        initializeModules();
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
            std::bind(&SensorFusionNode::imuCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fastlio/odom", rclcpp::SensorDataQoS(),
            std::bind(&SensorFusionNode::odomCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&SensorFusionNode::scanCallback, this, std::placeholders::_1));
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom_filtered", rclcpp::SensorDataQoS());
        
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
        
        int publish_rate = this->get_parameter("publish_rate").as_int();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / publish_rate),
            std::bind(&SensorFusionNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "Sensor Fusion Node initialized at %d Hz", publish_rate);
    }

private:
    void initializeModules() {
        imu_prefilter_ = std::make_unique<sensor_fusion::ImuPrefilter>();
        
        double alpha = this->get_parameter("imu.complementary_gain").as_double();
        imu_prefilter_->setComplementaryFilterAlpha(alpha);
        imu_prefilter_->setBiasEstimationGain(0.001);
        
        ekf_ = std::make_unique<sensor_fusion::MultiRateEKF2D>();
        
        double q_pos = this->get_parameter("ekf.process_noise_pos").as_double();
        double q_theta = this->get_parameter("ekf.process_noise_theta").as_double();
        double q_vel = this->get_parameter("ekf.process_noise_vel").as_double();
        double q_omega = this->get_parameter("ekf.process_noise_omega").as_double();
        ekf_->setProcessNoise(q_pos, q_theta, q_vel, q_omega);
        
        double r_lidar_pos = this->get_parameter("ekf.lidar_noise_pos").as_double();
        double r_lidar_theta = this->get_parameter("ekf.lidar_noise_theta").as_double();
        ekf_->setLidarNoise(r_lidar_pos, r_lidar_theta);
        
        double r_wheel = this->get_parameter("ekf.wheel_noise_vel").as_double();
        ekf_->setWheelNoise(r_wheel);
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        
        sensor_fusion::ImuMeasurement imu_meas;
        imu_meas.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        imu_meas.accel = Eigen::Vector3d(
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
        );
        imu_meas.gyro = Eigen::Vector3d(
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z
        );
        imu_meas.mag = Eigen::Vector3d::Zero();
        
        imu_prefilter_->processImu(imu_meas);
        
        latest_imu_ = imu_meas;
        latest_attitude_ = imu_prefilter_->getAttitude();
        last_imu_time_ = imu_meas.timestamp;
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(lidar_mutex_);
        
        sensor_fusion::LidarMeasurement2D lidar_meas;
        lidar_meas.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        lidar_meas.x = msg->pose.pose.position.x;
        lidar_meas.y = msg->pose.pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        lidar_meas.theta = yaw;
        
        lidar_meas.confidence = 1.0;
        
        ekf_->updateLidar(lidar_meas);
        
        last_lidar_time_ = lidar_meas.timestamp;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr) {
    }
    
    void timerCallback() {
        auto now = this->now();
        double current_time = now.seconds();
        
        if (!ekf_initialized_) {
            initializeEKF();
            return;
        }
        
        double dt = current_time - last_predict_time_;
        if (dt <= 0.0 || dt > 0.1) {
            last_predict_time_ = current_time;
            return;
        }
        
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            
            if (last_imu_time_ > 0.0) {
                sensor_fusion::ImuMeasurement2D imu_2d;
                imu_2d.timestamp = latest_imu_.timestamp;
                
                Eigen::Matrix3d R = latest_attitude_.toRotationMatrix();
                Eigen::Vector3d accel_body = R.transpose() * latest_imu_.accel;
                accel_body.z() = 0.0;
                
                imu_2d.ax = accel_body.x();
                imu_2d.ay = accel_body.y();
                imu_2d.omega = latest_imu_.gyro.z();
                
                ekf_->predict(imu_2d, dt);
            }
        }
        
        auto state = ekf_->getState();
        publishOdometry(state, now);
        publishTransform(state, now);
        
        last_predict_time_ = current_time;
    }
    
    void initializeEKF() {
        auto state = ekf_->getState();
        
        if (std::abs(state.x) < 1e-6 && std::abs(state.y) < 1e-6) {
            ekf_initialized_ = true;
            last_predict_time_ = this->now().seconds();
            RCLCPP_INFO(this->get_logger(), "EKF initialized");
        }
    }
    
    void publishOdometry(const sensor_fusion::State2D& state, const rclcpp::Time& now) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        
        odom.pose.pose.position.x = state.x;
        odom.pose.pose.position.y = state.y;
        odom.pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, state.theta);
        odom.pose.pose.orientation = tf2::toMsg(q);
        
        odom.twist.twist.linear.x = state.vx;
        odom.twist.twist.linear.y = state.vy;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = state.omega;
        
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                odom.pose.covariance[i * 6 + j] = state.cov(i, j);
            }
        }
        
        odom_pub_->publish(odom);
    }
    
    void publishTransform(const sensor_fusion::State2D& state, const rclcpp::Time& now) {
        if (!this->get_parameter("publish_tf").as_bool()) {
            return;
        }
        
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = now;
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        
        tf.transform.translation.x = state.x;
        tf.transform.translation.y = state.y;
        tf.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, state.theta);
        tf.transform.rotation = tf2::toMsg(q);
        
        tf_broadcaster_->sendTransform(tf);
    }
    
    std::unique_ptr<sensor_fusion::MultiRateEKF2D> ekf_;
    std::unique_ptr<sensor_fusion::ImuPrefilter> imu_prefilter_;
    
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::mutex imu_mutex_;
    std::mutex lidar_mutex_;
    
    sensor_fusion::ImuMeasurement latest_imu_;
    Eigen::Quaterniond latest_attitude_;
    double last_imu_time_ = 0.0;
    double last_lidar_time_ = 0.0;
    
    bool ekf_initialized_ = false;
    double last_predict_time_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorFusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
