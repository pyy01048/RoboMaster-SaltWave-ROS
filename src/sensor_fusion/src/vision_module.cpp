#include "sensor_fusion/vision_module.hpp"
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace sensor_fusion {

VisionModule::VisionModule()
    : current_pose_(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), 0.0) {
    orb_detector_ = cv::ORB::create(500);
    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING);
    
    camera_matrix_ << 800, 0, 320,
                       0, 800, 240,
                       0, 0, 1;
}

void VisionModule::setCameraMatrix(const Eigen::Matrix3d& K) {
    std::lock_guard<std::mutex> lock(mutex_);
    camera_matrix_ = K;
}

void VisionModule::setLandmarks(const std::vector<Eigen::Vector3d>& landmarks) {
    std::lock_guard<std::mutex> lock(mutex_);
    landmarks_ = landmarks;
}

std::vector<cv::KeyPoint> VisionModule::detectFeatures(const cv::Mat& image) {
    std::vector<cv::KeyPoint> keypoints;
    orb_detector_->detect(image, keypoints);
    return keypoints;
}

std::vector<VisualFeature> VisionModule::matchFeatures(
    const std::vector<cv::KeyPoint>& keypoints,
    const cv::Mat& descriptors) {
    
    std::vector<VisualFeature> features;
    if (landmarks_.empty() || keypoints.empty()) {
        return features;
    }
    
    cv::Mat landmark_descriptors;
    for (const auto& lm : landmarks_) {
        landmark_descriptors.push_back(cv::Mat::zeros(1, 32, CV_8U));
    }
    
    std::vector<std::vector<cv::DMatch>> matches;
    matcher_->knnMatch(descriptors, landmark_descriptors, matches, 2);
    
    for (const auto& m : matches) {
        if (m.size() >= 2 && m[0].distance < 0.75 * m[1].distance) {
            VisualFeature feature;
            feature.image_point = keypoints[m[0].queryIdx].pt;
            feature.world_point = landmarks_[m[0].trainIdx];
            feature.landmark_id = m[0].trainIdx;
            features.push_back(feature);
        }
    }
    
    return features;
}

VisualPose VisionModule::solvePnPRansac(const std::vector<VisualFeature>& features) {
    VisualPose pose;
    pose.confidence = 0.0;
    
    if (features.size() < 4) {
        return pose;
    }
    
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;
    
    for (const auto& f : features) {
        object_points.push_back(cv::Point3f(f.world_point.x(), f.world_point.y(), f.world_point.z()));
        image_points.push_back(f.image_point);
    }
    
    cv::Mat camera_mat = (cv::Mat_<double>(3, 3) << 
        camera_matrix_(0, 0), camera_matrix_(0, 1), camera_matrix_(0, 2),
        camera_matrix_(1, 0), camera_matrix_(1, 1), camera_matrix_(1, 2),
        camera_matrix_(2, 0), camera_matrix_(2, 1), camera_matrix_(2, 2));
    
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);
    
    cv::Mat rvec, tvec;
    bool success = cv::solvePnPRansac(object_points, image_points, camera_mat, 
                                      dist_coeffs, rvec, tvec, false, 100, 8.0, 0.99);
    
    if (success) {
        cv::Mat R_mat;
        cv::Rodrigues(rvec, R_mat);
        
        Eigen::Matrix3d R_eigen;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_eigen(i, j) = R_mat.at<double>(i, j);
            }
        }
        
        pose.position = Eigen::Vector3d(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        pose.orientation = Eigen::Quaterniond(R_eigen);
        pose.confidence = 1.0;
    }
    
    return pose;
}

double VisionModule::computeReprojectionError(
    const std::vector<VisualFeature>& features,
    const VisualPose& pose) {
    
    if (features.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    double total_error = 0.0;
    Eigen::Matrix3d R = pose.orientation.toRotationMatrix();
    
    for (const auto& f : features) {
        Eigen::Vector3d world_point = f.world_point;
        Eigen::Vector3d camera_point = R.transpose() * (world_point - pose.position);
        Eigen::Vector3d image_point_h = camera_matrix_ * (camera_point / camera_point.z());
        
        double dx = image_point_h.x() - f.image_point.x;
        double dy = image_point_h.y() - f.image_point.y;
        total_error += dx*dx + dy*dy;
    }
    
    return std::sqrt(total_error / features.size());
}

void VisionModule::processImage(const cv::Mat& image) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto keypoints = detectFeatures(image);
    if (keypoints.empty()) {
        current_pose_.confidence *= 0.8;
        return;
    }
    
    cv::Mat descriptors;
    orb_detector_->compute(image, keypoints, descriptors);
    
    auto features = matchFeatures(keypoints, descriptors);
    if (features.size() < 4) {
        current_pose_.confidence *= 0.9;
        return;
    }
    
    auto pose = solvePnPRansac(features);
    if (pose.confidence > 0.0) {
        current_pose_ = pose;
        current_features_ = features;
    }
}

VisualPose VisionModule::getCurrentPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_pose_;
}

std::vector<VisualFeature> VisionModule::getFeatures() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_features_;
}

}
