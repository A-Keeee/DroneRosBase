#ifndef _ESKF_HPP_
#define _ESKF_HPP_

#include <Eigen/Dense>
#include <vector>

class ESKF
{
public:
    ESKF();
    ~ESKF();

    void predict(const Eigen::Vector3d& angular_velocity, const Eigen::Vector3d& linear_acceleration, double dt);
    void update(const Eigen::Vector3d& gps_position, const Eigen::Vector3d& imu_angular_velocity);

private:
    // State vector: [position(3), velocity(3), orientation(3), gyro_bias(3), accel_bias(3)]
    Eigen::Matrix<double, 15, 15> P; // Error covariance matrix
    Eigen::Matrix<double, 15, 15> F_x; // State transition matrix
    Eigen::Matrix<double, 
    Eigen::Matrix<double, 15, 15> Q; // Process noise covariance matrix
    Eigen::Matrix<double, 6, 6> R; // Measurement noise covariance matrix
    Eigen::Vector<double, 15> x; // State vector
    Eigen::Vector<double, 6> measurement; // Measurement vector
};

#endif // _ESKF_HPP_