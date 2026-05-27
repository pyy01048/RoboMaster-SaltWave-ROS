#include "sensor_fusion/multi_rate_ekf.hpp"
#include <iostream>

namespace sensor_fusion {

MultiRateEKF::MultiRateEKF() {
    initializeState();
    setProcessNoise(0.01, 0.1, 0.001);
    setMeasurementNoise(0.05, 0.01, 0.1);
}

void MultiRateEKF::initializeState() {
    state_.setZero();
    state_(6) = 1.0;
    P_ = Eigen::Matrix<double, 15, 15>::Identity() * 0.1;
}

void MultiRateEKF::setProcessNoise(double q_pos, double q_vel, double q_bias) {
    Q_ = Eigen::Matrix<double, 15, 15>::Zero();
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q_pos;
    Q_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * q_pos;
    Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * q_vel;
    Q_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * q_bias;
    Q_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * q_bias;
}

void MultiRateEKF::setMeasurementNoise(double r_lidar_pos, double r_lidar_rot, double r_wheel) {
    R_lidar_ = Eigen::Matrix<double, 6, 6>::Zero();
    R_lidar_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * r_lidar_pos;
    R_lidar_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * r_lidar_rot;
    
    R_wheel_ = Eigen::Matrix2d::Identity() * r_wheel;
}

Eigen::Matrix3d MultiRateEKF::skewSymmetric(const Eigen::Vector3d& v) const {
    Eigen::Matrix3d skew;
    skew << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return skew;
}

void MultiRateEKF::predict(const ImuData& imu_data, double dt) {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Vector3d pos = state_.segment<3>(0);
    Eigen::Vector3d vel = state_.segment<3>(3);
    Eigen::Quaterniond q(state_(6), state_(7), state_(8), state_(9));
    q.normalize();
    Eigen::Vector3d bias_acc = state_.segment<3>(10);
    Eigen::Vector3d bias_gyro = state_.segment<3>(12);

    Eigen::Vector3d accel_corrected = imu_data.accel - bias_acc;
    Eigen::Vector3d gyro_corrected = imu_data.gyro - bias_gyro;

    Eigen::Matrix3d R = q.toRotationMatrix();
    Eigen::Vector3d accel_world = R * accel_corrected;

    pos = pos + vel * dt + 0.5 * accel_world * dt * dt;
    vel = vel + accel_world * dt;

    Eigen::Vector3d delta_rot = gyro_corrected * dt;
    Eigen::Quaterniond dq;
    dq.w() = cos(0.5 * delta_rot.norm());
    dq.vec() = sin(0.5 * delta_rot.norm()) * delta_rot.normalized();
    q = q * dq;
    q.normalize();

    state_.segment<3>(0) = pos;
    state_.segment<3>(3) = vel;
    state_(6) = q.w();
    state_.segment<3>(7) = q.vec();

    Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    F.block<3, 3>(3, 6) = -R * skewSymmetric(accel_corrected) * dt;
    F.block<3, 3>(3, 10) = -R * dt;

    P_ = F * P_ * F.transpose() + Q_ * dt;
}

void MultiRateEKF::fuseLidar(const LidarOdomData& lidar_data) {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Vector3d pos_meas = lidar_data.position;
    Eigen::Quaterniond q_meas = lidar_data.orientation;
    q_meas.normalize();

    Eigen::VectorXd innovation(6);
    innovation.segment<3>(0) = pos_meas - state_.segment<3>(0);
    
    Eigen::Quaterniond q_est(state_(6), state_(7), state_(8), state_(9));
    Eigen::Quaterniond q_error = q_meas * q_est.inverse();
    innovation.segment<3>(3) = 2.0 * q_error.vec();

    Eigen::Matrix<double, 6, 15> H = Eigen::Matrix<double, 6, 15>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 15, 6> K = P_ * H.transpose() * (H * P_ * H.transpose() + R_lidar_).inverse();
    state_ = state_ + K * innovation;
    P_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P_;

    Eigen::Quaterniond q_updated(state_(6), state_(7), state_(8), state_(9));
    q_updated.normalize();
    state_(6) = q_updated.w();
    state_.segment<3>(7) = q_updated.vec();
}

void MultiRateEKF::fuseWheel(double left_vel, double right_vel, double dt) {
    std::lock_guard<std::mutex> lock(mutex_);

    double wheel_base = 0.4;
    double wheel_radius = 0.05;

    double linear_vel = (left_vel + right_vel) * wheel_radius / 2.0;
    double angular_vel = (right_vel - left_vel) * wheel_radius / wheel_base;

    Eigen::Vector2d measurement(linear_vel, angular_vel);
    Eigen::Vector2d prediction(state_(3), state_(4));

    Eigen::Vector2d innovation = measurement - prediction;

    Eigen::Matrix<double, 2, 15> H = Eigen::Matrix<double, 2, 15>::Zero();
    H(0, 3) = 1.0;
    H(1, 4) = 1.0;

    Eigen::Matrix<double, 15, 2> K = P_ * H.transpose() * (H * P_ * H.transpose() + R_wheel_).inverse();
    state_ = state_ + K * innovation;
    P_ = (Eigen::Matrix<double, 15, 15>::Identity() - K * H) * P_;
}

Eigen::Vector3d MultiRateEKF::getPosition() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.segment<3>(0);
}

Eigen::Quaterniond MultiRateEKF::getOrientation() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Eigen::Quaterniond q(state_(6), state_(7), state_(8), state_(9));
    q.normalize();
    return q;
}

Eigen::Vector3d MultiRateEKF::getVelocity() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.segment<3>(3);
}

}
