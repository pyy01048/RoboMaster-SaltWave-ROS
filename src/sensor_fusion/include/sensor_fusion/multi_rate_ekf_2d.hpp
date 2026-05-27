#pragma once

#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <cmath>

namespace sensor_fusion {

struct State2D {
    double x, y, theta;
    double vx, vy, omega;
    
    Eigen::Matrix<double, 6, 6> cov;
};

struct ImuMeasurement2D {
    double timestamp;
    double ax, ay;
    double omega;
};

struct LidarMeasurement2D {
    double timestamp;
    double x, y, theta;
    double confidence;
};

class MultiRateEKF2D {
public:
    MultiRateEKF2D();
    
    void predict(const ImuMeasurement2D& imu, double dt);
    void updateLidar(const LidarMeasurement2D& meas);
    void updateWheel(double vx_meas, double vy_meas, double dt);
    
    State2D getState() const;
    void setProcessNoise(double q_pos, double q_theta, double q_vel, double q_omega);
    void setLidarNoise(double r_pos, double r_theta);
    void setWheelNoise(double r_vel);

private:
    State2D state_;
    
    Eigen::Matrix<double, 6, 6> Q_;
    Eigen::Matrix3d R_lidar_;
    Eigen::Matrix2d R_wheel_;
    
    double last_time_;
    mutable std::mutex mutex_;
    
    double normalizeAngle(double angle) const;
    Eigen::Matrix<double, 6, 6> computeJacobian(double dt) const;
};

}
