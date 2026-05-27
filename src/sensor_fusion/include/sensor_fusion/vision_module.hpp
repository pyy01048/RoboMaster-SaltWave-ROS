#pragma once

#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace sensor_fusion {

struct VisualFeature {
    cv::Point2f image_point;
    Eigen::Vector3d world_point;
    int landmark_id;
};

struct VisualPose {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    double confidence;
    
    VisualPose() : position(Eigen::Vector3d::Zero()), orientation(Eigen::Quaterniond::Identity()), confidence(0.0) {}
    VisualPose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q, double c) : position(p), orientation(q), confidence(c) {}
};

class VisionModule {
public:
    VisionModule();
    ~VisionModule() = default;

    void processImage(const cv::Mat& image);
    void setCameraMatrix(const Eigen::Matrix3d& K);
    void setLandmarks(const std::vector<Eigen::Vector3d>& landmarks);
    
    VisualPose getCurrentPose() const;
    std::vector<VisualFeature> getFeatures() const;

private:
    Eigen::Matrix3d camera_matrix_;
    std::vector<Eigen::Vector3d> landmarks_;
    VisualPose current_pose_;
    std::vector<VisualFeature> current_features_;
    
    cv::Ptr<cv::ORB> orb_detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    
    mutable std::mutex mutex_;
    
    std::vector<cv::KeyPoint> detectFeatures(const cv::Mat& image);
    std::vector<VisualFeature> matchFeatures(
        const std::vector<cv::KeyPoint>& keypoints,
        const cv::Mat& descriptors);
    
    VisualPose solvePnPRansac(const std::vector<VisualFeature>& features);
    
    double computeReprojectionError(
        const std::vector<VisualFeature>& features,
        const VisualPose& pose);
};

}
