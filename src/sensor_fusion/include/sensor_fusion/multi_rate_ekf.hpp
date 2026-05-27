#pragma once

#include <Eigen/Dense>
#include <deque>
#include <mutex>

namespace sensor_fusion {

struct ImuData {
    double timestamp;
    Eigen::Vector3d accel;
    Eigen::Vector3d gyro;
};

struct LidarOdomData {
    double timestamp;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d linear_velocity;
};

class MultiRateEKF {
public:
    MultiRateEKF();
    ~MultiRateEKF() = default;

    void predict(const ImuData& imu_data, double dt);
    void fuseLidar(const LidarOdomData& lidar_data);
    void fuseWheel(double left_vel, double right_vel, double dt);

    Eigen::Vector3d getPosition() const;
    Eigen::Quaterniond getOrientation() const;
    Eigen::Vector3d getVelocity() const;

    void setProcessNoise(double q_pos, double q_vel, double q_bias);
    void setMeasurementNoise(double r_lidar_pos, double r_lidar_rot, double r_wheel);

private:
    Eigen::Matrix<double, 15, 1> state_;
    Eigen::Matrix<double, 15, 15> P_;
    Eigen::Matrix<double, 15, 15> Q_;
    Eigen::Matrix<double, 6, 6> R_lidar_;
    Eigen::Matrix<double, 2, 2> R_wheel_;

    mutable std::mutex mutex_;

    void initializeState();
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v) const;
};

}
