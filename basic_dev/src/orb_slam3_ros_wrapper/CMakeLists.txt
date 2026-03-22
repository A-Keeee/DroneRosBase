cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3_ros_wrapper)

add_compile_options(-std=c++14 -Wall -O3 -march=native)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  image_transport
  tf
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)


set(ORB_SLAM3_DIR /opt/ORB_SLAM3)

# 核心功能：导出 catkin 依赖项
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ORB_SLAM3_DIR}
  ${ORB_SLAM3_DIR}/include
  ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2
  ${ORB_SLAM3_DIR}/Thirdparty/g2o
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so
  ${ORB_SLAM3_DIR}/lib/libORB_SLAM3.so
  -lboost_system
  ${catkin_LIBRARIES}
)


add_executable(stereo_inertial_node src/stereo_inertial_node.cpp)

target_link_libraries(stereo_inertial_node ${LIBS})
