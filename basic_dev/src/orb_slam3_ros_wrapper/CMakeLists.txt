# ==============================================================================
# 核心功能：
# 配置 orb_slam3_ros_wrapper 功能包的编译规则，链接系统依赖及本地编译的 ORB_SLAM3 库。
# ==============================================================================

cmake_minimum_required(VERSION 3.0.2)
project(orb_slam3_ros_wrapper)

# 核心功能：设置 C++ 编译标准为 C++14（与之前我们修改的 ORB_SLAM3 及 Pangolin 保持一致），并开启优化选项
add_compile_options(-std=c++14 -Wall -O3 -march=native)

# ----------------- 步骤 1：寻找 ROS 与系统级依赖 -----------------
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

# ----------------- 步骤 2：定义 ORB_SLAM3 的基础路径 -----------------
# 关键变量：
# ORB_SLAM3_DIR: 指向与当前 ROS 包同级目录下的 ORB_SLAM3 源码文件夹
# 逻辑解释：使用 ${PROJECT_SOURCE_DIR}/.. 保证只要该功能包与 ORB_SLAM3 均在 src 目录下即可被识别
set(ORB_SLAM3_DIR ${PROJECT_SOURCE_DIR}/../ORB_SLAM3)

# 核心功能：导出 catkin 依赖项
catkin_package()

# ----------------- 步骤 3：配置包含目录 (头文件搜索路径) -----------------
# 逻辑解释：不仅要包含 ORB_SLAM3 自身的头文件，还必须包含它内置依赖的 Sophus、DBoW2、g2o 以及系统级的 Eigen3。
# 补全这些路径可以避免编译节点源码时出现 fatal error: g2o/core/base_vertex.h: No such file 等错误。
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

# ----------------- 步骤 4：整合所有需要链接的库文件 -----------------
# 关键变量：
# LIBS: 整合 OpenCV、Eigen3、Pangolin、ROS 系统库以及 ORB_SLAM3 编译输出的动态链接库 (.so) 的绝对路径
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

# ----------------- 步骤 5：编译节点并链接库 -----------------
# 核心功能：将 cpp 源文件编译为名为 stereo_inertial_node 的可执行程序
add_executable(stereo_inertial_node src/stereo_inertial_node.cpp)

# 核心功能：将整理好的 LIBS 库列表强制链接到刚刚生成的可执行文件上
target_link_libraries(stereo_inertial_node ${LIBS})