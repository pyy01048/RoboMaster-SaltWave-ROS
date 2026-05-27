#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <deque>
#include <mutex>
#include <Eigen/Dense>

class GraphOptimizerNode : public rclcpp::Node {
public:
    GraphOptimizerNode() : Node("graph_optimizer_node") {
        this->declare_parameter("lidar_odom_topic", "/Odometry");
        this->declare_parameter("imu_topic", "/imu_filtered");
        this->declare_parameter("optimized_odom_topic", "/odom_optimized");
        this->declare_parameter("loop_closure_distance", 2.0);
        this->declare_parameter("loop_closure_angle", 0.3);
        this->declare_parameter("graph_update_rate", 10.0);
        this->declare_parameter("graph_optimization_iterations", 5);

        lidar_odom_topic_ = this->get_parameter("lidar_odom_topic").as_string();
        imu_topic_ = this->get_parameter("imu_topic").as_string();
        optimized_odom_topic_ = this->get_parameter("optimized_odom_topic").as_string();
        loop_closure_distance_ = this->get_parameter("loop_closure_distance").as_double();
        loop_closure_angle_ = this->get_parameter("loop_closure_angle").as_double();
        graph_update_rate_ = this->get_parameter("graph_update_rate").as_double();
        graph_optimization_iterations_ = this->get_parameter("graph_optimization_iterations").as_int();

        lidar_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            lidar_odom_topic_, 1000,
            std::bind(&GraphOptimizerNode::lidar_odom_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 1000,
            std::bind(&GraphOptimizerNode::imu_callback, this, std::placeholders::_1));

        optimized_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(optimized_odom_topic_, 10);

        update_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / graph_update_rate_),
            std::bind(&GraphOptimizerNode::update_callback, this));

        RCLCPP_INFO(this->get_logger(), "Graph Optimizer Node started at %.2f Hz", graph_update_rate_);
    }

private:
    void lidar_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!first_odom_) {
            first_odom_ = true;
            last_optimized_pose_ = msg->pose.pose;
            first_optimized_time_ = msg->header.stamp;
        }

        keyframes_.push_back(*msg);
        if (keyframes_.size() > 1000) {
            keyframes_.pop_front();
        }

        detect_loop_closure();
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
    }

    void detect_loop_closure() {
        if (keyframes_.size() < 50) return;

        auto& current = keyframes_.back();
        for (size_t i = 0; i < keyframes_.size() - 50; ++i) {
            auto& kf = keyframes_[i];

            double dx = current.pose.pose.position.x - kf.pose.pose.position.x;
            double dy = current.pose.pose.position.y - kf.pose.pose.position.y;
            double dz = current.pose.pose.position.z - kf.pose.pose.position.z;
            double dist = sqrt(dx*dx + dy*dy + dz*dz);

            Eigen::Quaterniond q_curr(
                current.pose.pose.orientation.w,
                current.pose.pose.orientation.x,
                current.pose.pose.orientation.y,
                current.pose.pose.orientation.z);
            Eigen::Quaterniond q_kf(
                kf.pose.pose.orientation.w,
                kf.pose.pose.orientation.x,
                kf.pose.pose.orientation.y,
                kf.pose.pose.orientation.z);
            double angle_diff = q_curr.angularDistance(q_kf);

            if (dist < loop_closure_distance_ && angle_diff < loop_closure_angle_) {
                loop_closure_detected_ = true;
                RCLCPP_INFO(this->get_logger(), "Loop closure detected: dist=%.2f, angle=%.2f", dist, angle_diff);
                break;
            }
        }
    }

    void update_callback() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (keyframes_.empty()) return;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg = keyframes_.back();
        odom_msg.header.frame_id = "odom_optimized";
        odom_msg.child_frame_id = "base_link";

        optimized_odom_pub_->publish(odom_msg);

        if (loop_closure_detected_) {
            loop_closure_detected_ = false;
        }
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimized_odom_pub_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    std::deque<nav_msgs::msg::Odometry> keyframes_;
    geometry_msgs::msg::Pose last_optimized_pose_;
    rclcpp::Time first_optimized_time_;
    std::mutex mutex_;
    bool first_odom_ = false;
    bool loop_closure_detected_ = false;

    std::string lidar_odom_topic_;
    std::string imu_topic_;
    std::string optimized_odom_topic_;
    double loop_closure_distance_;
    double loop_closure_angle_;
    double graph_update_rate_;
    int graph_optimization_iterations_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GraphOptimizerNode>());
    rclcpp::shutdown();
    return 0;
}
