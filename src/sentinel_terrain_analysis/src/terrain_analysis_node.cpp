#include <iostream>
#include <algorithm>
#include <queue>
#include <omp.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Header.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace Eigen;

string output_dir = "";
string input_cloud_topic = "/livox/lidar";
int input_cloud_type = 0;
bool using_shenbei_mode = false;
bool using_robosense = false;
int N_SCANS = 6;
double sensorMinimumRange = 0.1;
double sensorMountAngle = 0;
int output_index = 0;
int output_width = 2000;
int output_height = 2000;
double output_resolution = 0.15;
bool show_opencv = true;
bool show_rviz = true;

double robot_height = 1.0;
double robot_clearance = 0.0;
double grid_size = 4.0;
double grid_num = (grid_size) / (output_resolution);

double grid_min_range = (grid_num / 2) * output_resolution;

double grid_center_x;
double grid_center_y;

double grid_min_y;
double grid_max_y;
double grid_min_x;
double grid_max_x;

int grid_index_min_y;
int grid_index_max_y;
int grid_index_min_x;
int grid_index_max_x;

double break_distance = 0.1;
bool use_break = false;

double break_distance_bias = 0.0;

bool use_filter = false;
double filter_threshold = 0.1;

double car_x, car_y, car_z;
double car_yaw;
double car_roll;
double car_pitch;

double pitch_tolerance = 0;
double roll_tolerance = 0;

int grid_counts[(int)((100.0 / 0.15) + 1)][(int)((100.0 / 0.15) + 1)];
int grid_counts_temp[(int)((100.0 / 0.15) + 1)][(int)((100.0 / 0.15) + 1)];

bool grid_visit[(int)((100.0 / 0.15) + 1)][(int)((100.0 / 0.15) + 1)];

int grid_visit_counts[(int)((100.0 / 0.15) + 1)];

int map_boundary = 2;

double intensity_thre = 100;

double max_obs_height = 3.0;

std::vector<pcl::PointXYZRGBNormal> points_to_visit;

std::vector<pcl::PointXYZRGBNormal> break_points;

std::vector<std::pair<double, double>> boundary_point;

int boundary_size = 0;

double boundary_x[(int)((100.0 / 0.15) + 1)];
double boundary_y[(int)((100.0 / 0.15) + 1)];

double max_x[(int)((100.0 / 0.15) + 1)][(int)((100.0 / 0.15) + 1)];
double min_x[(int)((100.0 / 0.15) + 1)][(int)((100.0 / 0.15) + 1)];
double max_y[(int)((100.0 / 0.15) + 1)][(int)((100.0 / 0.15) + 1)];
double min_y[(int)((100.0 / 0.15) + 1)][(int)((100.0 / 0.15) + 1)];

double obs_height[(int)((100.0 / 0.15) + 1)][(int)((100.0 / 0.15) + 1)];

ros::Publisher pub_break_points;
ros::Publisher pub_grid_map;

bool check_boundary(int x, int y)
{
  if (x < 0)
    return false;
  if (x >= output_width)
    return false;
  if (y < 0)
    return false;
  if (y >= output_height)
    return false;
  return true;
}
double get_height(int x, int y, cv::Mat &grid_map)
{
  int pixel = grid_map.at<uchar>(y, x);
  return double(pixel) / 255.0 * max_obs_height;
}
void reset_grid()
{
  for (int i = 0; i < (int)((grid_size / output_resolution) + 5); i++)
  {
    for (int j = 0; j < (int)((grid_size / output_resolution) + 5); j++)
    {
      grid_counts[i][j] = 0;
      grid_counts_temp[i][j] = 0;
      grid_visit[i][j] = false;
      obs_height[i][j] = 0;
    }
    grid_visit_counts[i] = 0;
  }
  break_points.clear();
  points_to_visit.clear();
}
int get_index(double value)
{
  return int((value + grid_min_range) / output_resolution);
}
int get_index(double value, int direction)
{
  if (direction)
    return int((value + grid_min_range) / output_resolution);
  else
    return int((value - grid_min_range) / output_resolution);
}
class TerrainAnalysisNode : public rclcpp::Node {
public:
  TerrainAnalysisNode() : Node("terrain_analysis_node") {
    // this->declare_parameter("input_cloud_topic", input_cloud_topic);
    // this->declare_parameter("input_cloud_type", input_cloud_type);
    this->declare_parameter("output_dir", output_dir);
    this->declare_parameter("using_shenbei_mode", using_shenbei_mode);
    this->declare_parameter("using_robosense", using_robosense);
    this->declare_parameter("N_SCANS", N_SCANS);
    this->declare_parameter("sensorMinimumRange", sensorMinimumRange);
    this->declare_parameter("sensorMountAngle", sensorMountAngle);
    this->declare_parameter("output_index", output_index);
    this->declare_parameter("output_width", output_width);
    this->declare_parameter("output_height", output_height);
    this->declare_parameter("output_resolution", output_resolution);
    this->declare_parameter("show_opencv", show_opencv);
    this->declare_parameter("show_rviz", show_rviz);
    this->declare_parameter("robot_height", robot_height);
    this->declare_parameter("robot_clearance", robot_clearance);
    this->declare_parameter("grid_size", grid_size);
    this->declare_parameter("break_distance", break_distance);
    this->declare_parameter("use_break", use_break);
    this->declare_parameter("break_distance_bias", break_distance_bias);
    this->declare_parameter("use_filter", use_filter);
    this->declare_parameter("filter_threshold", filter_threshold);
    this->declare_parameter("pitch_tolerance", pitch_tolerance);
    this->declare_parameter("roll_tolerance", roll_tolerance);
    this->declare_parameter("intensity_thre", intensity_thre);
    this->declare_parameter("max_obs_height", max_obs_height);

    // this->get_parameter("input_cloud_topic", input_cloud_topic);
    // this->get_parameter("input_cloud_type", input_cloud_type);
    this->get_parameter("output_dir", output_dir);
    this->get_parameter("using_shenbei_mode", using_shenbei_mode);
    this->get_parameter("using_robosense", using_robosense);
    this->get_parameter("N_SCANS", N_SCANS);
    this->get_parameter("sensorMinimumRange", sensorMinimumRange);
    this->get_parameter("sensorMountAngle", sensorMountAngle);
    this->get_parameter("output_index", output_index);
    this->get_parameter("output_width", output_width);
    this->get_parameter("output_height", output_height);
    this->get_parameter("output_resolution", output_resolution);
    this->get_parameter("show_opencv", show_opencv);
    this->get_parameter("show_rviz", show_rviz);
    this->get_parameter("robot_height", robot_height);
    this->get_parameter("robot_clearance", robot_clearance);
    this->get_parameter("grid_size", grid_size);
    this->get_parameter("break_distance", break_distance);
    this->get_parameter("use_break", use_break);
    this->get_parameter("break_distance_bias", break_distance_bias);
    this->get_parameter("use_filter", use_filter);
    this->get_parameter("filter_threshold", filter_threshold);
    this->get_parameter("pitch_tolerance", pitch_tolerance);
    this->get_parameter("roll_tolerance", roll_tolerance);
    this->get_parameter("intensity_thre", intensity_thre);
    this->get_parameter("max_obs_height", max_obs_height);

    std::cout << "break_distance: " << break_distance << std::endl;
    std::cout << "use_break: " << use_break << std::endl;
    std::cout << "break_distance_bias: " << break_distance_bias << std::endl;

    grid_num = (grid_size) / (output_resolution);
    grid_min_range = (grid_num / 2) * output_resolution;

    grid_center_x = grid_min_range;
    grid_center_y = grid_min_range;

    grid_min_y = -grid_center_y;
    grid_max_y = grid_center_y;
    grid_min_x = -grid_center_x;
    grid_max_x = grid_center_x;

    grid_index_min_y = grid_min_y / output_resolution;
    grid_index_max_y = grid_max_y / output_resolution;
    grid_index_min_x = grid_min_x / output_resolution;
    grid_index_max_x = grid_max_x / output_resolution;

    if (using_shenbei_mode)
    {
      sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/livox/lidar", 10, std::bind(&TerrainAnalysisNode::cloud_callback, this, std::placeholders::_1));
      sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/Odometry", 10, std::bind(&TerrainAnalysisNode::odom_callback, this, std::placeholders::_1));
    }
    else
    {
      sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/livox/lidar", 10, std::bind(&TerrainAnalysisNode::cloud_callback, this, std::placeholders::_1));
      sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
          "/state_estimation", 10, std::bind(&TerrainAnalysisNode::odom_callback, this, std::placeholders::_1));
    }

    pub_grid_map_ = this->create_publisher<sensor_msgs::msg::Image>("/grid_map", 10);
    pub_break_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/break_points", 10);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_grid_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_break_points_;

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (use_break && break_points.size() == 0)
    {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_lidar_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr non_ground_points(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr all_segments(new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    if (using_robosense)
    {
      sensor_msgs::PointCloud2 cloud_msg_out;
      if (input_cloud_type == 0)
      {
        pcl::PointCloud<pcl::PointXYZI> scan;
        pcl::fromROSMsg(*cloud_msg, scan);
        std::vector<pcl::PointCloud<pcl::PointXYZI>> scan_scans(N_SCANS);
        for (auto &point : scan.points)
        {
          double angle = atan(point.y / (point.x + 0.0001));
          if (angle >= -M_PI_2 && angle < M_PI_2)
          {
            int scanID = int((angle + M_PI_2) / (M_PI) * N_SCANS);
            if (scanID >= N_SCANS)
              scanID = N_SCANS - 1;
            if (scanID < 0)
              scanID = 0;
            scan_scans[scanID].points.push_back(point);
          }
        }
        std::string fields_list = "y";
        pcl::fromPCLPointCloud2(scan_scans[0].points[0].getPoint3().toPCL(), fields_list, cloud_msg_out);
      }
      else
      {
        sensor_msgs::PointCloud2 cloud_msg_out;
        std::string fields_list = "y";
        pcl::fromPCLPointCloud2(cloud_msg->points[0].getPoint3().toPCL(), fields_list, cloud_msg_out);
      }
      // pub_cloud.publish(cloud_msg_out);
      return;
    }

    pcl::fromROSMsg(*cloud_msg, *raw_lidar_points);
    cv::Mat grid_map(output_width, output_height, CV_8UC1, cv::Scalar(0));
    int cloud_size = raw_lidar_points->points.size();
    double point_time;
    for (int i = 0; i < cloud_size; i++)
    {
      double x = raw_lidar_points->points[i].x;
      double y = raw_lidar_points->points[i].y;
      double z = raw_lidar_points->points[i].z;

      // int rowIdn = raw_lidar_points->points[i].intensity;
      double rowIdn = raw_lidar_points->points[i].intensity;

      if (rowIdn >= N_SCANS / 2.)
        point_time = rowIdn;
      else
        point_time = rowIdn + N_SCANS;

      float start_ori = -atan2(raw_lidar_points->points[0].y, raw_lidar_points->points[0].x);
      float end_ori = -atan2(raw_lidar_points->points[cloud_size - 1].y,
                              raw_lidar_points->points[cloud_size - 1].x) +
                       2 * M_PI;

      if (end_ori - start_ori > 3 * M_PI)
      {
        end_ori -= 2 * M_PI;
      }
      else if (end_ori - start_ori < M_PI)
      {
        start_ori += 2 * M_PI;
      }

      bool halfPassed = false;
      int half_cloud_index = cloud_size / 2;
      for (int i = 0; i < cloud_size; i++)
      {
        double ori = -atan2(raw_lidar_points->points[i].y, raw_lidar_points->points[i].x);
        if (!halfPassed)
        {
          if (ori < start_ori - M_PI)
            ori += 2 * M_PI;
          else if (ori > start_ori + M_PI)
            ori -= 2 * M_PI;

          if (ori - start_ori > M_PI)
            halfPassed = true;
        }
        else
        {
          ori += 2 * M_PI;

          if (ori < end_ori - M_PI)
            ori += 2 * M_PI;
          else if (ori > end_ori + M_PI)
            ori -= 2 * M_PI;
        }
      }

      int index = (int)round((point_time / N_SCANS) * cloud_size);
      double time = index / (double)cloud_size;
      // time = (double)i / cloud_size;

      double distance = sqrt(x * x + y * y + z * z);
      if (distance < sensorMinimumRange)
        continue;

      double pitch, roll, yaw;

      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
      transform.rotate(Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitX()));
      transform.rotate(Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitY()));
      transform.rotate(Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()));
      transform.translation() << car_x, car_y, car_z;

      pcl::PointXYZRGBNormal pointRGBN;
      pointRGBN.x = x;
      pointRGBN.y = y;
      pointRGBN.z = z;
      pcl::PointXYZRGBNormal point_after;
      point_after = pcl::transformPoint(pointRGBN, transform);
      x = point_after.x;
      y = point_after.y;
      z = point_after.z;

      double x_rel = x - car_x;
      double y_rel = y - car_y;
      double z_rel = z - car_z;

      double yaw_angle;
      yaw_angle = atan2(y_rel, x_rel);

      double pitch_rel;
      pitch_rel = atan2(z_rel, sqrt(x_rel * x_rel + y_rel * y_rel));
      // double dist = sqrt(x_rel*x_rel + y_rel*y_rel + z_rel*z_rel);

      yaw_angle += car_yaw;
      pitch_rel -= car_pitch;

      x_rel = cos(yaw_angle) * sqrt(x_rel * x_rel + y_rel * y_rel + z_rel * z_rel);
      y_rel = sin(yaw_angle) * sqrt(x_rel * x_rel + y_rel * y_rel + z_rel * z_rel);
      z_rel = sin(pitch_rel) * sqrt(x_rel * x_rel + y_rel * y_rel + z_rel * z_rel);
      x_rel = cos(pitch_rel) * sqrt(x_rel * x_rel + y_rel * y_rel + z_rel * z_rel);

      x = x_rel + car_x;
      y = y_rel + car_y;
      z = z_rel + car_z;

      if (sqrt(x_rel * x_rel + y_rel * y_rel) > grid_max_x)
        continue;
      int x_index, y_index;
      x_index = int((x + grid_min_range) / output_resolution);
      y_index = int((y + grid_min_range) / output_resolution);

      if (x_index < 0 || x_index >= output_width)
        continue;
      if (y_index < 0 || y_index >= output_height)
        continue;

      if (z - car_z > robot_height)
        continue;

      // double obsHeight = z - car_z;
      double obsHeight = z;

      if (grid_counts[x_index][y_index] == 0)
      {
        obs_height[x_index][y_index] = obsHeight;
      }
      else
      {
        if (obsHeight < obs_height[x_index][y_index])
          obs_height[x_index][y_index] = obsHeight;
      }
      grid_counts[x_index][y_index]++;

      pointRGBN.x = x;
      pointRGBN.y = y;
      pointRGBN.z = z;
      pointRGBN.curvature = time;

      if (obsHeight < robot_height + robot_clearance)
      {
        ground_points->points.push_back(pointRGBN);
        continue;
      }
      non_ground_points->points.push_back(pointRGBN);
    }

    for (int i = 0; i < output_width; i++)
      for (int j = 0; j < output_height; j++)
        grid_counts_temp[i][j] = grid_counts[i][j];

    for (int i = 0; i < output_width; i++)
    {
      for (int j = 0; j < output_height; j++)
      {
        if (grid_counts_temp[i][j] == 0)
        {
          continue;
        }
        if (obs_height[i][j] > robot_height + robot_clearance)
        {
          continue;
        }
        for (int x = i - map_boundary; x <= i + map_boundary; x++)
        {
          for (int y = j - map_boundary; y <= j + map_boundary; y++)
          {
            if (check_boundary(x, y))
            {
              if (obs_height[x][y] > obs_height[i][j])
              {
                obs_height[x][y] = obs_height[i][j];
                grid_counts[x][y] = grid_counts[i][j];
                grid_visit[x][y] = false;
              }
            }
          }
        }
      }
    }

    int ground_segment_num = ground_points->points.size();
    for (int i = 0; i < ground_segment_num; i++)
    {
      double x = ground_points->points[i].x;
      double y = ground_points->points[i].y;
      double z = ground_points->points[i].z;

      int x_index, y_index;
      x_index = int((x + grid_min_range) / output_resolution);
      y_index = int((y + grid_min_range) / output_resolution);

      if (x_index < 0 || x_index >= output_width)
        continue;
      if (y_index < 0 || y_index >= output_height)
        continue;

      if (z - obs_height[x_index][y_index] > 0.05)
      {
        non_ground_points->points.push_back(ground_points->points[i]);
        continue;
      }
      ground_points->points[i].z = obs_height[x_index][y_index];
    }

    for (int i = 0; i < output_width; i++)
    {
      for (int j = 0; j < output_height; j++)
      {
        if (grid_counts[i][j] > 0)
        {
          grid_counts[i][j] = 1;
        }
      }
    }

    for (int i = 0; i < output_width; i++)
    {
      for (int j = 0; j < output_height; j++)
      {
        if (grid_counts[i][j] == 1)
        {
          bool ground_neighbors = false;
          for (int x = i - map_boundary; x <= i + map_boundary; x++)
          {
            for (int y = j - map_boundary; y <= j + map_boundary; y++)
            {
              if (check_boundary(x, y))
              {
                if (grid_counts[x][y] == 1)
                {
                  if (obs_height[x][y] <= robot_height + robot_clearance)
                  {
                    ground_neighbors = true;
                  }
                }
              }
            }
          }
          if (ground_neighbors)
          {
            grid_counts[i][j] = 2;
          }
        }
      }
    }

    for (int i = 0; i < output_width; i++)
    {
      for (int j = 0; j < output_height; j++)
      {
        if (grid_counts[i][j] == 2)
        {
          for (int x = i - map_boundary; x <= i + map_boundary; x++)
          {
            for (int y = j - map_boundary; y <= j + map_boundary; y++)
            {
              if (check_boundary(x, y))
              {
                if (obs_height[x][y] > obs_height[i][j])
                {
                  obs_height[x][y] = obs_height[i][j];
                }
              }
            }
          }
        }
      }
    }

    std::vector<pcl::PointXYZRGBNormal> non_ground_points_new;
    int non_ground_segment_num = non_ground_points->points.size();
    for (int i = 0; i < non_ground_segment_num; i++)
    {
      double x = non_ground_points->points[i].x;
      double y = non_ground_points->points[i].y;
      double z = non_ground_points->points[i].z;

      int x_index, y_index;
      x_index = int((x + grid_min_range) / output_resolution);
      y_index = int((y + grid_min_range) / output_resolution);

      if (x_index < 0 || x_index >= output_width)
        continue;
      if (y_index < 0 || y_index >= output_height)
        continue;

      if (z - obs_height[x_index][y_index] < 0.05)
      {
        ground_points->points.push_back(non_ground_points->points[i]);
      }
      else
      {
        non_ground_points_new.push_back(non_ground_points->points[i]);
      }
    }
    non_ground_points->points.clear();
    for (int i = 0; i < non_ground_points_new.size(); i++)
    {
      non_ground_points->points.push_back(non_ground_points_new[i]);
    }

    for (int i = 0; i < output_width; i++)
    {
      for (int j = 0; j < output_height; j++)
      {
        if (grid_counts[i][j] == 2)
        {
          int pixel = int(obs_height[i][j] / max_obs_height * 255);
          if (pixel > 255)
            pixel = 255;
          grid_map.at<uchar>(i, j) = pixel;
        }
      }
    }

    for (int i = 0; i < ground_points->points.size(); i++)
    {
      all_segments->points.push_back(ground_points->points[i]);
    }
    for (int i = 0; i < non_ground_points->points.size(); i++)
    {
      all_segments->points.push_back(non_ground_points->points[i]);
    }

    sensor_msgs::msg::PointCloud2::Ptr segmented_cloud(new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*all_segments, *segmented_cloud);
    segmented_cloud->header = cloud_msg->header;
    segmented_cloud->header.frame_id = "map";
    sensor_msgs::msg::ImagePtr img_msg = cv_bridge::CvImage(segmented_cloud->header, sensor_msgs::image_encodings::TYPE_8UC1, grid_map).toImageMsg();
    pub_grid_map_->publish(*img_msg);

    int ground_size = ground_points->points.size();
    for (int i = 0; i < ground_size; i++)
    {
      double x = ground_points->points[i].x;
      double y = ground_points->points[i].y;
      double z = ground_points->points[i].z;

      int x_index, y_index;
      x_index = int((x + grid_min_range) / output_resolution);
      y_index = int((y + grid_min_range) / output_resolution);

      if (x_index < 0 || x_index >= output_width)
        continue;
      if (y_index < 0 || y_index >= output_height)
        continue;
      if (grid_counts[x_index][y_index] != 2)
        continue;

      double roll, pitch, yaw;
      roll = -asin(2 * (ground_points->points[i].normal_y * ground_points->points[i].normal_z -
                         ground_points->points[i].normal_x * ground_points->points[i].normal_curvature));
      pitch = atan2(2 * (ground_points->points[i].normal_x * ground_points->points[i].normal_y +
                         ground_points->points[i].normal_z * ground_points->points[i].normal_curvature),
                    1 - 2 * (ground_points->points[i].normal_y * ground_points->points[i].normal_y +
                             ground_points->points[i].normal_z * ground_points->points[i].normal_z));
      yaw = atan2(2 * (ground_points->points[i].normal_x * ground_points->points[i].normal_z +
                       ground_points->points[i].normal_y * ground_points->points[i].normal_curvature),
                  1 - 2 * (ground_points->points[i].normal_z * ground_points->points[i].normal_z +
                           ground_points->points[i].normal_curvature * ground_points->points[i].normal_curvature));

      double roll_angle, pitch_angle;

      Eigen::Vector3d normal(ground_points->points[i].normal_x,
                             ground_points->points[i].normal_y,
                             ground_points->points[i].normal_z);

      double distance;
      distance = -normal(0) * (x - car_x) - normal(1) * (y - car_y) - normal(2) * (z - car_z);
      if (distance < 0)
      {
        continue;
      }

      if (abs(roll) > roll_tolerance || abs(pitch) > pitch_tolerance)
      {
        pcl::PointXYZRGBNormal pointRGBN;
        pointRGBN.x = x;
        pointRGBN.y = y;
        pointRGBN.z = z;
        pointRGBN.curvature = ground_points->points[i].curvature;
        ground_points->points[i].r = 255;
        ground_points->points[i].g = 0;
        ground_points->points[i].b = 0;
        non_ground_points->points.push_back(pointRGBN);
        continue;
      }
    }

    std::vector<pcl::PointXYZRGBNormal> ground_points_new;
    ground_size = ground_points->points.size();
    for (int i = 0; i < ground_size; i++)
    {
      if (ground_points->points[i].r != 255)
      {
        ground_points_new.push_back(ground_points->points[i]);
      }
    }
    ground_points->points.clear();
    for (int i = 0; i < ground_points_new.size(); i++)
    {
      ground_points->points.push_back(ground_points_new[i]);
    }

    for (int i = 0; i < ground_points->points.size(); i++)
    {
      all_segments->points.push_back(ground_points->points[i]);
    }
    for (int i = 0; i < non_ground_points->points.size(); i++)
    {
      all_segments->points.push_back(non_ground_points->points[i]);
    }

    if (show_opencv)
    {
      // cv::namedWindow("Grid map", cv::WINDOW_AUTOSIZE);
      // cv::imshow("Grid map", grid_map);
      // cv::waitKey(10);
    }

    pcl::toROSMsg(*all_segments, *segmented_cloud);
    segmented_cloud->header = cloud_msg->header;
    segmented_cloud->header.frame_id = "map";
    // pub_segmented_cloud.publish(*segmented_cloud);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    car_x = odom_msg->pose.pose.position.x;
    car_y = odom_msg->pose.pose.position.y;
    car_z = odom_msg->pose.pose.position.z;

    Eigen::Quaternionf q(odom_msg->pose.pose.orientation.w,
                         odom_msg->pose.pose.orientation.x,
                         odom_msg->pose.pose.orientation.y,
                         odom_msg->pose.pose.orientation.z);
    Eigen::Matrix3f R = q.toRotationMatrix();
    double roll, pitch, yaw;
    roll = atan2(R(2, 1), R(2, 2));
    pitch = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
    yaw = atan2(R(1, 0), R(0, 0));

    car_yaw = yaw;
    car_pitch = pitch;
    car_roll = roll;
  }

  std::mutex mtx_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TerrainAnalysisNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
