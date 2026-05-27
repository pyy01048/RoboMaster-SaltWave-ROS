#include "sensor_fusion/lidar_module.hpp"
#include <cmath>
#include <algorithm>

namespace sensor_fusion {

LidarModule::LidarModule()
    : max_correspondence_distance_(0.5)
    , outlier_threshold_(0.05)
    , max_iterations_(50) {
    current_pose_.position = Eigen::Vector3d::Zero();
    current_pose_.orientation = Eigen::Quaterniond::Identity();
    current_pose_.confidence = 0.0;
}

void LidarModule::setMaxCorrespondenceDistance(double dist) {
    max_correspondence_distance_ = std::max(0.01, dist);
}

void LidarModule::setOutlierThreshold(double thresh) {
    outlier_threshold_ = std::max(0.001, thresh);
}

void LidarModule::setMaxIterations(int iter) {
    max_iterations_ = std::max(1, iter);
}

std::vector<Point3D> LidarModule::filterOutliers(const std::vector<Point3D>& points) {
    std::vector<Point3D> filtered;
    filtered.reserve(points.size());
    
    for (const auto& p : points) {
        double dist = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (dist > 0.1 && dist < 30.0) {
            filtered.push_back(p);
        }
    }
    
    return filtered;
}

std::vector<std::pair<Point3D, Point3D>> LidarModule::findCorrespondences(
    const std::vector<Point3D>& source, 
    const std::vector<Point3D>& target) {
    
    std::vector<std::pair<Point3D, Point3D>> correspondences;
    correspondences.reserve(source.size());
    
    for (const auto& s : source) {
        double min_dist = std::numeric_limits<double>::max();
        Point3D best_target;
        
        for (const auto& t : target) {
            double dx = s.x - t.x;
            double dy = s.y - t.y;
            double dz = s.z - t.z;
            double dist = dx*dx + dy*dy + dz*dz;
            
            if (dist < min_dist) {
                min_dist = dist;
                best_target = t;
            }
        }
        
        if (std::sqrt(min_dist) < max_correspondence_distance_) {
            correspondences.push_back({s, best_target});
        }
    }
    
    return correspondences;
}

Eigen::Matrix4d LidarModule::icpAlignment(
    const std::vector<std::pair<Point3D, Point3D>>& correspondences) {
    
    if (correspondences.size() < 3) {
        return Eigen::Matrix4d::Identity();
    }
    
    Eigen::Vector3d centroid_source = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid_target = Eigen::Vector3d::Zero();
    
    for (const auto& corr : correspondences) {
        centroid_source.x() += corr.first.x;
        centroid_source.y() += corr.first.y;
        centroid_source.z() += corr.first.z;
        centroid_target.x() += corr.second.x;
        centroid_target.y() += corr.second.y;
        centroid_target.z() += corr.second.z;
    }
    
    centroid_source /= correspondences.size();
    centroid_target /= correspondences.size();
    
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (const auto& corr : correspondences) {
        Eigen::Vector3d s(corr.first.x - centroid_source.x(),
                        corr.first.y - centroid_source.y(),
                        corr.first.z - centroid_source.z());
        Eigen::Vector3d t(corr.second.x - centroid_target.x(),
                        corr.second.y - centroid_target.y(),
                        corr.second.z - centroid_target.z());
        H += s * t.transpose();
    }
    
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
    
    if (R.determinant() < 0) {
        Eigen::Matrix3d V = svd.matrixV();
        V.col(2) *= -1;
        R = V * svd.matrixU().transpose();
    }
    
    Eigen::Vector3d t = centroid_target - R * centroid_source;
    
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) = R;
    transform.block<3, 1>(0, 3) = t;
    
    return transform;
}

double LidarModule::computeError(
    const std::vector<std::pair<Point3D, Point3D>>& correspondences,
    const Eigen::Matrix4d& transform) {
    
    double error = 0.0;
    Eigen::Matrix3d R = transform.block<3, 3>(0, 0);
    Eigen::Vector3d t = transform.block<3, 1>(0, 3);
    
    for (const auto& corr : correspondences) {
        Eigen::Vector3d s(corr.first.x, corr.first.y, corr.first.z);
        Eigen::Vector3d t_gt(corr.second.x, corr.second.y, corr.second.z);
        Eigen::Vector3d transformed = R * s + t;
        error += (transformed - t_gt).squaredNorm();
    }
    
    return error / correspondences.size();
}

void LidarModule::processScan(const LidarScan& scan) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto filtered_points = filterOutliers(scan.points);
    if (filtered_points.empty()) {
        return;
    }
    
    if (scan_history_.empty()) {
        scan_history_.push_back(scan);
        current_pose_.confidence = 1.0;
        return;
    }
    
    const auto& reference_scan = scan_history_.back();
    auto correspondences = findCorrespondences(filtered_points, reference_scan.points);
    
    if (correspondences.size() < 10) {
        current_pose_.confidence *= 0.5;
        return;
    }
    
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    double prev_error = std::numeric_limits<double>::max();
    
    for (int i = 0; i < max_iterations_; ++i) {
        auto current_correspondences = findCorrespondences(filtered_points, reference_scan.points);
        if (current_correspondences.size() < 10) break;
        
        Eigen::Matrix4d delta_transform = icpAlignment(current_correspondences);
        transform = delta_transform * transform;
        
        double error = computeError(current_correspondences, transform);
        if (std::abs(prev_error - error) < 1e-6) break;
        prev_error = error;
    }
    
    Eigen::Matrix3d R = transform.block<3, 3>(0, 0);
    Eigen::Vector3d t = transform.block<3, 1>(0, 3);
    
    Eigen::Quaterniond delta_q(R);
    current_pose_.position = R * current_pose_.position + t;
    current_pose_.orientation = current_pose_.orientation * delta_q;
    current_pose_.orientation.normalize();
    
    current_pose_.confidence = std::min(1.0, current_pose_.confidence + 0.01);
    
    scan_history_.push_back(scan);
    if (scan_history_.size() > 10) {
        scan_history_.pop_front();
    }
}

LidarPose LidarModule::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_pose_;
}

}
