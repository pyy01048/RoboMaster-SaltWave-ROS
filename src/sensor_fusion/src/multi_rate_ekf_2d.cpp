#include "sensor_fusion/multi_rate_ekf_2d.hpp"
#include <iostream>

namespace sensor_fusion {

MultiRateEKF2D::MultiRateEKF2D() : last_time_(0.0) {
    state_.x = state_.y = state_.theta = 0.0;
    state_.vx = state_.vy = state_.omega = 0.0;
    state_.cov = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
    
    setProcessNoise(0.01, 0.001, 0.1, 0.01);
    setLidarNoise(0.05, 0.01);
    setWheelNoise(0.1);
}

double MultiRateEKF2D::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

void MultiRateEKF2D::setProcessNoise(double q_pos, double q_theta, double q_vel, double q_omega) {
    Q_ = Eigen::Matrix<double, 6, 6>::Zero();
    Q_(0, 0) = q_pos;
    Q_(1, 1) = q_pos;
    Q_(2, 2) = q_theta;
    Q_(3, 3) = q_vel;
    Q_(4, 4) = q_vel;
    Q_(5, 5) = q_omega;
}

void MultiRateEKF2D::setLidarNoise(double r_pos, double r_theta) {
    R_lidar_ = Eigen::Matrix3d::Zero();
    R_lidar_(0, 0) = r_pos;
    R_lidar_(1, 1) = r_pos;
    R_lidar_(2, 2) = r_theta;
}

void MultiRateEKF2D::setWheelNoise(double r_vel) {
    R_wheel_ = Eigen::Matrix2d::Identity() * r_vel;
}

Eigen::Matrix<double, 6, 6> MultiRateEKF2D::computeJacobian(double dt) const {
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    
    double c = std::cos(state_.theta);
    double s = std::sin(state_.theta);
    
    F(0, 2) = (-state_.vx * s - state_.vy * c) * dt;
    F(0, 3) = c * dt;
    F(0, 4) = -s * dt;
    
    F(1, 2) = (state_.vx * c - state_.vy * s) * dt;
    F(1, 3) = s * dt;
    F(1, 4) = c * dt;
    
    F(2, 5) = dt;
    
    return F;
}

void MultiRateEKF2D::predict(const ImuMeasurement2D& imu, double dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (dt <= 0.0 || dt > 0.1) return;
    
    double c = std::cos(state_.theta);
    double s = std::sin(state_.theta);
    
    state_.x += (state_.vx * c - state_.vy * s) * dt;
    state_.y += (state_.vx * s + state_.vy * c) * dt;
    state_.theta += state_.omega * dt;
    state_.theta = normalizeAngle(state_.theta);
    
    state_.vx += imu.ax * dt;
    state_.vy += imu.ay * dt;
    state_.omega = imu.omega;
    
    auto F = computeJacobian(dt);
    state_.cov = F * state_.cov * F.transpose() + Q_ * dt;
}

void MultiRateEKF2D::updateLidar(const LidarMeasurement2D& meas) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    Eigen::Vector3d innovation;
    innovation(0) = meas.x - state_.x;
    innovation(1) = meas.y - state_.y;
    innovation(2) = normalizeAngle(meas.theta - state_.theta);
    
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;
    
    Eigen::Matrix3d S = H * state_.cov * H.transpose() + R_lidar_ / meas.confidence;
    Eigen::Matrix<double, 6, 3> K = state_.cov * H.transpose() * S.inverse();
    
    Eigen::Matrix<double, 6, 1> innovation_full;
    innovation_full << innovation(0), innovation(1), innovation(2), 0, 0, 0;
    
    state_.x += K(0, 0) * innovation(0) + K(0, 1) * innovation(1) + K(0, 2) * innovation(2);
    state_.y += K(1, 0) * innovation(0) + K(1, 1) * innovation(1) + K(1, 2) * innovation(2);
    state_.theta += K(2, 0) * innovation(0) + K(2, 1) * innovation(1) + K(2, 2) * innovation(2);
    state_.theta = normalizeAngle(state_.theta);
    
    state_.cov = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * state_.cov;
}

void MultiRateEKF2D::updateWheel(double vx_meas, double vy_meas, double dt) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    Eigen::Vector2d innovation;
    innovation(0) = vx_meas - state_.vx;
    innovation(1) = vy_meas - state_.vy;
    
    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
    H(0, 3) = 1.0;
    H(1, 4) = 1.0;
    
    Eigen::Matrix2d S = H * state_.cov * H.transpose() + R_wheel_;
    Eigen::Matrix<double, 6, 2> K = state_.cov * H.transpose() * S.inverse();
    
    state_.vx += K(3, 0) * innovation(0) + K(3, 1) * innovation(1);
    state_.vy += K(4, 0) * innovation(0) + K(4, 1) * innovation(1);
    
    state_.cov = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * state_.cov;
}

State2D MultiRateEKF2D::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

}
