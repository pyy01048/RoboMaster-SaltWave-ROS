#include "sensor_fusion/ekf_fusion.hpp"
#include <cmath>

namespace sensor_fusion {

EKFFusion::EKFFusion(const Config& config) : config_(config) {
    state_.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 10.0;
    state_.position = Eigen::Vector2d::Zero();
    state_.theta = 0.0;
    state_.velocity = Eigen::Vector2d::Zero();
    state_.angular_velocity = 0.0;
}

void EKFFusion::initialize(const Pose2D& initial_pose, 
                          const Velocity2D& initial_velocity) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    state_.position = initial_pose.position;
    state_.theta = initial_pose.theta;
    state_.velocity = initial_velocity.velocity;
    state_.angular_velocity = 0.0;
    
    state_.covariance.block<2, 2>(0, 0) = 
        Eigen::Matrix2d::Identity() * config_.process_noise_position;
    state_.covariance.block<2, 2>(3, 3) = 
        Eigen::Matrix2d::Identity() * config_.process_noise_velocity;
    state_.covariance(2, 2) = config_.process_noise_angle;
    state_.covariance(5, 5) = config_.process_noise_angular_vel;
    
    initialized_ = true;
    last_update_time_ = 0.0;
}

void EKFFusion::predict(double dt) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!initialized_ || dt <= 0) return;
    
    propagateState(dt);
    
    Eigen::Matrix<double, 6, 6> F = computeProcessJacobian(dt);
    Eigen::Matrix<double, 6, 6> Q = computeProcessCovariance(dt);
    
    state_.covariance = F * state_.covariance * F.transpose() + Q;
    
    last_update_time_ += dt;
}

void EKFFusion::updateLidar(const Pose2D& measured_pose, double timestamp) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!initialized_) return;
    
    if (last_lidar_time_ >= 0 && timestamp - last_lidar_time_ > config_.max_lidar_age) {
        return;
    }
    
    Eigen::Vector3d z;
    z << measured_pose.position.x(), measured_pose.position.y(), measured_pose.theta;
    
    Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;
    
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    R(0, 0) = config_.lidar_noise_position;
    R(1, 1) = config_.lidar_noise_position;
    R(2, 2) = config_.lidar_noise_angle;
    
    Eigen::Vector3d y = z - hPosition(state_);
    y(2) = normalizeAngle(measured_pose.theta - state_.theta);
    
    Eigen::Matrix3d S = H * state_.covariance * H.transpose() + R;
    Eigen::Matrix<double, 6, 3> K = state_.covariance * H.transpose() * S.inverse();
    
    state_.position += K.block<2, 3>(0, 0) * y;
    state_.theta += K(2, 0) * y(0) + K(2, 1) * y(1) + K(2, 2) * y(2);
    state_.theta = normalizeAngle(state_.theta);
    state_.velocity += K.block<2, 3>(3, 0) * y;
    state_.angular_velocity += K(5, 0) * y(0) + K(5, 1) * y(1) + K(5, 2) * y(2);
    
    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    state_.covariance = (I - K * H) * state_.covariance;
    
    last_lidar_time_ = timestamp;
}

void EKFFusion::updateImu(double angular_velocity, double roll, 
                         double pitch, double timestamp) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!initialized_) return;
    
    if (last_imu_time_ >= 0 && timestamp - last_imu_time_ > config_.max_imu_age) {
        return;
    }
    
    Eigen::Matrix<double, 1, 1> z;
    z(0, 0) = angular_velocity;
    
    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
    H(0, 5) = 1.0;
    
    Eigen::Matrix<double, 1, 1> R;
    R(0, 0) = config_.imu_noise_angular_vel;
    
    Eigen::Matrix<double, 1, 1> y;
    y(0, 0) = angular_velocity - state_.angular_velocity;
    
    Eigen::Matrix<double, 1, 1> S = H * state_.covariance * H.transpose() + R;
    Eigen::Matrix<double, 6, 1> K = state_.covariance * H.transpose() / S(0, 0);
    
    state_.angular_velocity += K(5, 0) * y(0, 0);
    
    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    state_.covariance = (I - K * H) * state_.covariance;
    
    last_imu_time_ = timestamp;
}

void EKFFusion::updateVision(const Pose2D& measured_pose, double timestamp) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!initialized_) return;
    
    if (last_vision_time_ >= 0 && timestamp - last_vision_time_ > config_.max_vision_age) {
        return;
    }
    
    Eigen::Vector2d z = measured_pose.position;
    
    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * config_.vision_noise_position;
    
    Eigen::Vector2d y = z - hPosition(state_);
    
    Eigen::Matrix2d S = H * state_.covariance * H.transpose() + R;
    Eigen::Matrix<double, 6, 2> K = state_.covariance * H.transpose() * S.inverse();
    
    state_.position += K.block<2, 2>(0, 0) * y;
    state_.velocity += K.block<2, 2>(3, 0) * y;
    
    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    state_.covariance = (I - K * H) * state_.covariance;
    
    last_vision_time_ = timestamp;
}

void EKFFusion::updateEncoder(const Velocity2D& measured_velocity, double timestamp) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!initialized_) return;
    
    if (last_encoder_time_ >= 0 && timestamp - last_encoder_time_ > config_.max_encoder_age) {
        return;
    }
    
    Eigen::Vector2d z = measured_velocity.velocity;
    
    Eigen::Matrix<double, 2, 6> H = Eigen::Matrix<double, 2, 6>::Zero();
    H(0, 3) = 1.0;
    H(1, 4) = 1.0;
    
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * config_.encoder_noise_velocity;
    
    Eigen::Vector2d y = z - hVelocity(state_);
    
    Eigen::Matrix2d S = H * state_.covariance * H.transpose() + R;
    Eigen::Matrix<double, 6, 2> K = state_.covariance * H.transpose() * S.inverse();
    
    state_.velocity += K.block<2, 2>(3, 0) * y;
    
    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    state_.covariance = (I - K * H) * state_.covariance;
    
    last_encoder_time_ = timestamp;
}

EKFFusion::State EKFFusion::getState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

Pose2D EKFFusion::getPose() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    Pose2D pose;
    pose.position = state_.position;
    pose.theta = state_.theta;
    return pose;
}

Velocity2D EKFFusion::getVelocity() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    Velocity2D vel;
    vel.velocity = state_.velocity;
    return vel;
}

Eigen::Matrix2d EKFFusion::getPositionCovariance() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_.covariance.block<2, 2>(0, 0);
}

void EKFFusion::reset() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.covariance = Eigen::Matrix<double, 6, 6>::Identity() * 10.0;
    state_.position = Eigen::Vector2d::Zero();
    state_.theta = 0.0;
    state_.velocity = Eigen::Vector2d::Zero();
    state_.angular_velocity = 0.0;
    initialized_ = false;
}

void EKFFusion::propagateState(double dt) {
    double c = std::cos(state_.theta);
    double s = std::sin(state_.theta);
    
    state_.position.x() += (state_.velocity.x() * c - state_.velocity.y() * s) * dt;
    state_.position.y() += (state_.velocity.x() * s + state_.velocity.y() * c) * dt;
    state_.theta += state_.angular_velocity * dt;
    state_.theta = normalizeAngle(state_.theta);
}

Eigen::Matrix<double, 6, 6> EKFFusion::computeProcessJacobian(double dt) {
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    
    double c = std::cos(state_.theta);
    double s = std::sin(state_.theta);
    
    F(0, 2) = (-state_.velocity.x() * s - state_.velocity.y() * c) * dt;
    F(0, 3) = c * dt;
    F(0, 4) = -s * dt;
    
    F(1, 2) = (state_.velocity.x() * c - state_.velocity.y() * s) * dt;
    F(1, 3) = s * dt;
    F(1, 4) = c * dt;
    
    F(2, 5) = dt;
    
    return F;
}

Eigen::Matrix<double, 6, 6> EKFFusion::computeProcessCovariance(double dt) {
    Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
    Q(0, 0) = config_.process_noise_position * dt;
    Q(1, 1) = config_.process_noise_position * dt;
    Q(2, 2) = config_.process_noise_angle * dt;
    Q(3, 3) = config_.process_noise_velocity * dt;
    Q(4, 4) = config_.process_noise_velocity * dt;
    Q(5, 5) = config_.process_noise_angular_vel * dt;
    return Q;
}

Eigen::Vector2d EKFFusion::hPosition(const State& state) const {
    return state.position;
}

Eigen::Vector2d EKFFusion::hVelocity(const State& state) const {
    return state.velocity;
}

double EKFFusion::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

}
