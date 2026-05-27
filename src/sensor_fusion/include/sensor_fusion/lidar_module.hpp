#pragma once

#include <Eigen/Dense>
#include <vector>
#include <deque>
#include <mutex>

namespace sensor_fusion {

struct Point3D {
    double x, y, z;
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct LidarScan {
    double timestamp;
    std::vector<Point3D> points;
};

struct LidarPose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    double confidence;
};

class LidarModule {
public:
    LidarModule();
    ~LidarModule() = default;

    void processScan(const LidarScan& scan);
    LidarPose getCurrentPose() const;
    
    void setMaxCorrespondenceDistance(double dist);
    void setOutlierThreshold(double thresh);
    void setMaxIterations(int iter);

private:
    std::deque<LidarScan> scan_history_;
    LidarPose current_pose_;
    
    double max_correspondence_distance_;
    double outlier_threshold_;
    int max_iterations_;
    
    mutable std::mutex mutex_;
    
    std::vector<std::pair<Point3D, Point3D>> findCorrespondences(
        const std::vector<Point3D>& source, 
        const std::vector<Point3D>& target);
    
    Eigen::Matrix4d icpAlignment(
        const std::vector<std::pair<Point3D, Point3D>>& correspondences);
    
    double computeError(
        const std::vector<std::pair<Point3D, Point3D>>& correspondences,
        const Eigen::Matrix4d& transform);
    
    std::vector<Point3D> filterOutliers(const std::vector<Point3D>& points);
};

}
