#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

/**
 * @brief EKF惯导融合节点
 * 
 * 功能：
 * 1. 订阅IMU原始数据
 * 2. 订阅FAST-LIO里程计数据
 * 3. 发布融合后的Odometry数据
 * 4. 发布TF变换 (odom -> base_link)
 */
class EkfFusionNode : public rclcpp::Node
{
public:
  EkfFusionNode()
  : Node("ekf_fusion_node")
  {
    // 声明参数
    this->declare_parameter("imu_topic", "/livox/imu");
    this->declare_parameter("fast_lio_odom_topic", "/Odometry");
    this->declare_parameter("fused_odom_topic", "/odom");
    this->declare_parameter("publish_rate", 50.0);
    this->declare_parameter("use_imu", true);
    this->declare_parameter("use_fast_lio", true);

    // 获取参数
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    fast_lio_odom_topic_ = this->get_parameter("fast_lio_odom_topic").as_string();
    fused_odom_topic_ = this->get_parameter("fused_odom_topic").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    use_imu_ = this->get_parameter("use_imu").as_bool();
    use_fast_lio_ = this->get_parameter("use_fast_lio").as_bool();

    // 创建订阅者
    if (use_imu_) {
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10,
        std::bind(&EkfFusionNode::imuCallback, this, std::placeholders::_1));
    }

    if (use_fast_lio_) {
      fast_lio_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        fast_lio_odom_topic_, 10,
        std::bind(&EkfFusionNode::fastLioOdomCallback, this, std::placeholders::_1));
    }

    // 创建发布者
    fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(fused_odom_topic_, 10);

    // 创建TF广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 创建定时器，以指定频率发布融合后的里程计
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&EkfFusionNode::timerCallback, this));

    // 初始化状态
    current_pose_.position.x = 0.0;
    current_pose_.position.y = 0.0;
    current_pose_.position.z = 0.0;
    current_pose_.orientation.w = 1.0;
    current_pose_.orientation.x = 0.0;
    current_pose_.orientation.y = 0.0;
    current_pose_.orientation.z = 0.0;

    current_twist_.linear.x = 0.0;
    current_twist_.linear.y = 0.0;
    current_twist_.linear.z = 0.0;
    current_twist_.angular.x = 0.0;
    current_twist_.angular.y = 0.0;
    current_twist_.angular.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "EKF Fusion Node initialized");
    RCLCPP_INFO(this->get_logger(), "IMU topic: %s", imu_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "FAST-LIO odom topic: %s", fast_lio_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Fused odom topic: %s", fused_odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publish rate: %.2f Hz", publish_rate_);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 保存最新的IMU数据
    last_imu_msg_ = msg;
  }

  void fastLioOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 保存最新的FAST-LIO里程计数据
    last_fast_lio_odom_msg_ = msg;

    // 更新当前状态
    current_pose_ = msg->pose.pose;
    current_twist_ = msg->twist.twist;
    odom_frame_ = msg->header.frame_id;
    base_frame_ = msg->child_frame_id;
  }

  void timerCallback()
  {
    auto fused_odom = nav_msgs::msg::Odometry();
    fused_odom.header.stamp = this->now();
    fused_odom.header.frame_id = odom_frame_.empty() ? "odom" : odom_frame_;
    fused_odom.child_frame_id = base_frame_.empty() ? "base_link" : base_frame_;

    // 融合姿态和角速度（优先使用IMU数据）
    if (last_imu_msg_ != nullptr && use_imu_) {
      fused_odom.pose.pose.orientation = last_imu_msg_->orientation;
      fused_odom.twist.twist.angular = last_imu_msg_->angular_velocity;
    } else {
      fused_odom.pose.pose.orientation = current_pose_.orientation;
      fused_odom.twist.twist.angular = current_twist_.angular;
    }

    // 融合位置和线速度（优先使用FAST-LIO数据）
    if (use_fast_lio_) {
      fused_odom.pose.pose.position = current_pose_.position;
      fused_odom.twist.twist.linear = current_twist_.linear;
    }

    // 发布融合后的里程计
    fused_odom_pub_->publish(fused_odom);

    // 发布TF变换
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = odom_frame_.empty() ? "odom" : odom_frame_;
    t.child_frame_id = base_frame_.empty() ? "base_link" : base_frame_;

    t.transform.translation.x = fused_odom.pose.pose.position.x;
    t.transform.translation.y = fused_odom.pose.pose.position.y;
    t.transform.translation.z = fused_odom.pose.pose.position.z;
    t.transform.rotation = fused_odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fast_lio_odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Imu::SharedPtr last_imu_msg_;
  nav_msgs::msg::Odometry::SharedPtr last_fast_lio_odom_msg_;

  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Twist current_twist_;
  std::string odom_frame_;
  std::string base_frame_;

  std::string imu_topic_;
  std::string fast_lio_odom_topic_;
  std::string fused_odom_topic_;
  double publish_rate_;
  bool use_imu_;
  bool use_fast_lio_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfFusionNode>());
  rclcpp::shutdown();
  return 0;
}
