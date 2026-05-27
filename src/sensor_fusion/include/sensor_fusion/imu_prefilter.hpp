#pragma once

#include <Eigen/Dense>
#include <deque>
#include <mutex>

namespace sensor_fusion {

struct ImuMeasurement {
    double timestamp;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
    Eigen::Vector3d mag;
};

class ImuPrefilter {
public:
    ImuPrefilter();
    ~ImuPrefilter() = default;

    void processImu(const ImuMeasurement& meas);
    Eigen::Quaterniond getAttitude() const;
    Eigen::Vector3d getBiasAccel() const;
    Eigen::Vector3d getBiasGyro() const;
    
    void setComplementaryFilterAlpha(double alpha);
    void setBiasEstimationGain(double gain);

private:
    Eigen::Quaterniond attitude_;
    Eigen::Vector3d bias_accel_;
    Eigen::Vector3d bias_gyro_;
    
    double complementary_alpha_;
    double bias_estimation_gain_;
    
    double last_timestamp_;
    bool initialized_;
    
    mutable std::mutex mutex_;
    
    Eigen::Vector3d accelToEuler(const Eigen::Vector3d& accel) const;
    Eigen::Vector3d magToYaw(const Eigen::Vector3d& mag, const Eigen::Quaterniond& attitude) const;
};

}
