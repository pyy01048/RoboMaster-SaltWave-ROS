#include "sensor_fusion/imu_prefilter.hpp"
#include <cmath>

namespace sensor_fusion {

ImuPrefilter::ImuPrefilter()
    : complementary_alpha_(0.95)
    , bias_estimation_gain_(0.001)
    , last_timestamp_(0.0)
    , initialized_(false) {
    attitude_ = Eigen::Quaterniond::Identity();
    bias_accel_ = Eigen::Vector3d::Zero();
    bias_gyro_ = Eigen::Vector3d::Zero();
}

void ImuPrefilter::setComplementaryFilterAlpha(double alpha) {
    complementary_alpha_ = std::max(0.0, std::min(1.0, alpha));
}

void ImuPrefilter::setBiasEstimationGain(double gain) {
    bias_estimation_gain_ = std::max(0.0, gain);
}

Eigen::Vector3d ImuPrefilter::accelToEuler(const Eigen::Vector3d& accel) const {
    Eigen::Vector3d euler;
    euler.x() = std::atan2(accel.y(), accel.z());
    euler.y() = std::atan2(-accel.x(), std::sqrt(accel.y()*accel.y() + accel.z()*accel.z()));
    euler.z() = 0.0;
    return euler;
}

Eigen::Vector3d ImuPrefilter::magToYaw(const Eigen::Vector3d& mag, const Eigen::Quaterniond& attitude) const {
    Eigen::Matrix3d R = attitude.toRotationMatrix();
    Eigen::Vector3d mag_world = R * mag.normalized();
    return Eigen::Vector3d(0, 0, std::atan2(mag_world.y(), mag_world.x()));
}

void ImuPrefilter::processImu(const ImuMeasurement& meas) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!initialized_) {
        Eigen::Vector3d accel_norm = meas.accel.normalized();
        Eigen::Vector3d euler = accelToEuler(accel_norm);
        attitude_ = Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX());
        last_timestamp_ = meas.timestamp;
        initialized_ = true;
        return;
    }
    
    double dt = meas.timestamp - last_timestamp_;
    if (dt <= 0.0 || dt > 0.1) {
        last_timestamp_ = meas.timestamp;
        return;
    }
    
    Eigen::Vector3d gyro_corrected = meas.gyro - bias_gyro_;
    Eigen::Vector3d delta_angle = gyro_corrected * dt;
    Eigen::Quaterniond dq;
    double angle_norm = delta_angle.norm();
    if (angle_norm < 1e-6) {
        dq = Eigen::Quaterniond::Identity();
    } else {
        dq = Eigen::Quaterniond(Eigen::AngleAxisd(angle_norm, delta_angle.normalized()));
    }
    
    Eigen::Quaterniond attitude_gyro = attitude_ * dq;
    
    Eigen::Vector3d accel_norm = meas.accel.normalized();
    Eigen::Vector3d euler_accel = accelToEuler(accel_norm);
    Eigen::Quaterniond attitude_accel(euler_accel.z(), euler_accel.y(), euler_accel.x(), 0);
    attitude_accel.normalize();
    
    attitude_.w() = complementary_alpha_ * attitude_gyro.w() + (1 - complementary_alpha_) * attitude_accel.w();
    attitude_.vec() = complementary_alpha_ * attitude_gyro.vec() + (1 - complementary_alpha_) * attitude_accel.vec();
    attitude_.normalize();
    
    bias_gyro_ += bias_estimation_gain_ * gyro_corrected * dt;
    bias_accel_ += bias_estimation_gain_ * (meas.accel - bias_accel_) * dt;
    
    last_timestamp_ = meas.timestamp;
}

Eigen::Quaterniond ImuPrefilter::getAttitude() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return attitude_;
}

Eigen::Vector3d ImuPrefilter::getBiasAccel() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return bias_accel_;
}

Eigen::Vector3d ImuPrefilter::getBiasGyro() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return bias_gyro_;
}

}
