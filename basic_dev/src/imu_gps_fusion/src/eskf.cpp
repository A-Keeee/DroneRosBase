#include "eskf.hpp"

ESKF::ESKF()
{
    P.setIdentity();
    x.setZero();
}

ESKF::~ESKF()
{
}

void ESKF::predict(const Eigen::Vector3d& angular_velocity, const Eigen::Vector3d& linear_acceleration, double dt)
{
    
}

void ESKF::update(const Eigen::Vector3d& gps_position, const Eigen::Vector3d& imu_angular_velocity)
{
    
}
