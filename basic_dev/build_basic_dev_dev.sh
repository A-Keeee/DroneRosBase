#!/bin/bash

set -e

cd /basic_dev
source /opt/ros/noetic/setup.bash

catkin_make --only-pkg-with-deps airsim_ros
source devel/setup.bash
catkin_make --only-pkg-with-deps basic_dev
source devel/setup.bash
catkin_make --only-pkg-with-deps imu_gps_odometry
source devel/setup.bash
catkin_make --only-pkg-with-deps controller_test
source devel/setup.bash
catkin_make --only-pkg-with-deps odometry
source devel/setup.bash
catkin_make --only-pkg-with-deps my_orb_slam

echo "Build complete: airsim_ros, basic_dev, imu_gps_odometry, controller_test, odometry, my_orb_slam"