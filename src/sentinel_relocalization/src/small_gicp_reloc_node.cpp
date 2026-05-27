#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace Eigen;

bool save_map_path_global = true;
int map_increment_counter = 0;

class RelocalizationNode : public rclcpp::Node {
public:
  RelocalizationNode() : Node("small_gicp_reloc_node") {
    this->declare_parameter("pcd_file", "/home/pyy/Documents/sentiel/map/map.pcd");
    this->declare_parameter("save_dir", "/home/pyy/Documents/sentiel/map/map_update");
    this->get_parameter("pcd_file", pcd_file_);
    this->get_parameter("save_dir", save_dir_);

    path_pub_ = this->create_publisher<nav_msgs::Path>("/map_path", rclcpp::QoS(10));

    sub_map_ = this->create_subscription<nav_msgs::OccupancyGrid>(
        "/global_map", 10, std::bind(&RelocalizationNode::globalMapCallback, this, std::placeholders::_1));
  }
private:
  string pcd_file_;
  string save_dir_;
  rclcpp::Publisher<nav_msgs::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::OccupancyGrid>::SharedPtr sub_map_;
  std::mutex lock_;
  nav_msgs::Path map_path_;

  void globalMapCallback(const nav_msgs::OccupancyGrid::SharedPtr global_map_msg) {
    geometry_msgs::PoseStamped p;
    p.header = global_map_msg->header;
    p.pose = global_map_msg->origin;
    map_path_.poses.push_back(p);
    map_path_.header = global_map_msg->header;
    path_pub_->publish(map_path_);

    std::vector<float> path_x;
    std::vector<float> path_y;
    int size = map_path_.poses.size();

    for (int i = 0; i < size; i++) {
      path_x.push_back(map_path_.poses[i].pose.position.x);
      path_y.push_back(map_path_.poses[i].pose.position.y);
    }

    std::string pcd_save_path = save_dir_ + "/path";
    std::string pcd_path = pcd_file_;
    // std::string pcd_save_path = pcd_path.substr(0, pcd_path.size() - 4);

    std::ofstream outFile_x;
    std::ofstream outFile_y;

    outFile_x.open(pcd_save_path + ".x", std::ios::out);
    outFile_y.open(pcd_save_path + ".y", std::ios::out);

    if (!outFile_x.is_open())
      std::cout << "x.txt failed to open!" << std::endl;
    else {
      for (int i = 0; i < size; ++i) {
        outFile_x << path_x[i] << " " << std::endl;
      }
    }

    if (!outFile_y.is_open())
      std::cout << "y.txt failed to open!" << std::endl;
    else {
      for (int i = 0; i < size; ++i) {
        outFile_y << path_y[i] << " " << std::endl;
      }
    }
    outFile_x.close();
    outFile_y.close();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RelocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
