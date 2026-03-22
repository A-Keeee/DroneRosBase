# ==============================================================================
# 核心功能：
# 构建一个包含基础ROS Noetic环境、Pangolin可视化库、全局ORB_SLAM3算法库
# 以及用户自定义ROS功能包的完整Docker镜像。
# ==============================================================================

FROM osrf/ros:noetic-desktop-full-focal

# ----------------- 步骤 1：拷贝本地源代码 -----------------
# 逻辑解释：
# 1. 拷贝 ROS 工作空间
# 2. 拷贝 Pangolin 到根目录
# 3. 直接将 ORB_SLAM3 拷贝到全局系统目录 /opt/ORB_SLAM3 中
ADD src /basic_dev/src/
ADD Pangolin /Pangolin/
ADD ORB_SLAM3 /opt/ORB_SLAM3/
ADD setup.bash /

RUN chmod +x /setup.bash
USER root

# ----------------- 步骤 2：安装基础环境和依赖 -----------------
RUN apt update && apt install -y \
  python3-catkin-tools ros-noetic-geographic-msgs \
  ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs \
  ros-noetic-image-transport ros-noetic-cv-bridge \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  libopencv-dev ros-noetic-libg2o libsuitesparse-dev net-tools \
  wget cmake gcc g++ libglew-dev libeigen3-dev python3-dev python3-numpy git \
  libepoxy-dev pkg-config

ENV ROS_DISTRO=noetic

# ----------------- 步骤 3：编译与安装 Pangolin -----------------
WORKDIR /Pangolin
RUN find . -type f \( -name "CMakeLists.txt" -o -name "*.cmake" \) -exec sed -i 's/-Werror[a-zA-Z0-9=-]*//g' {} + && \
    cmake -B build -DCMAKE_CXX_FLAGS="-w" -DCMAKE_C_FLAGS="-w" && \
    cmake --build build -j$(nproc) && \
    cd build && make install && \
    ldconfig

# ----------------- 步骤 4：编译全局 ORB_SLAM3 核心库 -----------------
# 逻辑解释：直接在 /opt/ORB_SLAM3 下操作，将其固化为系统级依赖
WORKDIR /opt/ORB_SLAM3
RUN sed -i '1i #include <unistd.h>' src/LocalMapping.cc src/LoopClosing.cc src/System.cc src/Tracking.cc src/Viewer.cc || true && \
    find . -type f -name "CMakeLists.txt" -exec sed -i 's/OpenCV 4.4/OpenCV 4.2/g' {} + && \
    find . -type f -name "CMakeLists.txt" -exec sed -i 's/c++11/c++14/g' {} + && \
    mkdir -p Vocabulary && \
    sed -i 's/tar -xf ORBvoc.txt.tar.gz/tar -xf ORBvoc.txt.tar.gz || true/g' build.sh && \
    chmod +x build.sh && \
    ./build.sh

# ----------------- 步骤 5：编译用户的 ROS 工作空间 -----------------
WORKDIR /basic_dev/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin_make --only-pkg-with-deps airsim_ros && \
  . devel/setup.sh && catkin_make --only-pkg-with-deps basic_dev && \
  . devel/setup.sh && catkin_make --only-pkg-with-deps imu_gps_odometry && \
  . devel/setup.sh && catkin_make --only-pkg-with-deps controller_test && \
  . devel/setup.sh && catkin_make --only-pkg-with-deps orb_slam3_ros_wrapper && \
  . devel/setup.sh && catkin_make --only-pkg-with-deps odometry

ENTRYPOINT [ "/setup.bash" ]
