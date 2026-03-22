# ==============================================================================
# 核心功能：
# 构建一个包含基础ROS Noetic环境、Pangolin可视化计算库、ORB_SLAM3算法核心库
# 以及用户自定义ROS功能包的完整Docker镜像。
# ==============================================================================

# 基础镜像配置：使用官方提供的ROS Noetic桌面完整版，基于Ubuntu 20.04 (Focal)
FROM osrf/ros:noetic-desktop-full-focal

# ----------------- 步骤 1：拷贝本地源代码和脚本 -----------------
# 逻辑解释：将宿主机当前目录下的 src 文件夹（包含已解压的Pangolin、ORB_SLAM3及ROS包）
# 以及 setup.bash 启动脚本拷贝进容器内的指定目录。
ADD src /basic_dev/src/
ADD setup.bash /

# 核心功能：赋予容器内的 setup.bash 脚本可执行权限，作为容器启动入口
RUN chmod +x /setup.bash

# 切换为 root 用户以获取后续软件安装和系统级修改的权限
USER root

# ----------------- 步骤 2：安装基础环境和依赖 -----------------
# 逻辑解释：通过 apt 包管理器更新软件源并安装编译所需的各项依赖。
# 关键变量：新增了编译所需的基础工具链 (gcc, g++, cmake) 以及 Pangolin 和 ORB_SLAM3 的底层库 (libglew-dev, libeigen3-dev, libepoxy-dev 等)。
RUN apt update && apt install -y \
  python3-catkin-tools \
  ros-noetic-geographic-msgs \
  ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs \
  ros-noetic-image-transport ros-noetic-cv-bridge \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  libopencv-dev ros-noetic-libg2o libsuitesparse-dev net-tools \
  wget cmake gcc g++ libglew-dev libeigen3-dev python3-dev python3-numpy git \
  libepoxy-dev pkg-config

# 关键变量：声明当前系统使用的ROS发行版本名称（使用官方推荐的 key=value 格式）
ENV ROS_DISTRO=noetic

# ----------------- 步骤 3：编译与安装 Pangolin -----------------
# 逻辑解释：Pangolin 是 ORB_SLAM3 的前置依赖，必须先于 ORB_SLAM3 编译。
WORKDIR /basic_dev/src/Pangolin
# 核心功能：基于CMake构建Pangolin，并彻底清理严格警告标志。
# 重要逻辑步骤解释：
# 1. 优化正则表达式 's/-Werror[a-zA-Z0-9=-]*//g'：不仅匹配 -Werror，连带 -Werror=vla 这类后缀一起删除，防止产生残留的非法编译器参数。
# 2. make install：将编译输出的头文件和动态链接库安装到系统默认路径。
RUN find . -type f \( -name "CMakeLists.txt" -o -name "*.cmake" \) -exec sed -i 's/-Werror[a-zA-Z0-9=-]*//g' {} + && \
    cmake -B build -DCMAKE_CXX_FLAGS="-w" -DCMAKE_C_FLAGS="-w" && \
    cmake --build build -j$(nproc) && \
    cd build && make install

# ----------------- 步骤 4：编译 ORB_SLAM3 核心库 -----------------
# 逻辑解释：利用 ORB_SLAM3 官方提供的脚本完成自身及内嵌第三方库（DBoW2, g2o）的编译。
WORKDIR /basic_dev/src/ORB_SLAM3
# 核心功能：预先注入头文件、解除OpenCV版本限制、升级C++标准，随后执行整体编译。
# 重要逻辑步骤解释：
# 1. 注入 <unistd.h>：解决 Ubuntu 20.04 中 usleep() 未声明的报错。
# 2. 降级 OpenCV 需求：将强制的 OpenCV 4.4 替换为 4.2，匹配 ROS Noetic 自带环境。
# 3. 升级 C++ 标准：查找所有 CMakeLists.txt，将 C++11 替换为 C++14，避免 OpenCV 4 和新版 Pangolin 的语法冲突导致 System.cc 编译失败。
# 4. 执行官方脚本完成编译，生成 libORB_SLAM3.so。
RUN sed -i '1i #include <unistd.h>' src/LocalMapping.cc src/LoopClosing.cc src/System.cc src/Tracking.cc src/Viewer.cc || true && \
    find . -type f -name "CMakeLists.txt" -exec sed -i 's/OpenCV 4.4/OpenCV 4.2/g' {} + && \
    find . -type f -name "CMakeLists.txt" -exec sed -i 's/c++11/c++14/g' {} + && \
    chmod +x build.sh && \
    ./build.sh

# ----------------- 步骤 5：编译用户的 ROS 工作空间 -----------------
# 逻辑解释：返回上层工作空间目录，利用 catkin_make 工具分步独立编译用户提供的各个ROS包。
# 重要逻辑步骤解释：每次编译新包前都要通过 '. devel/setup.sh' 刷新环境变量，使得后编译的包能够正确链接前面已编译的包。
WORKDIR /basic_dev/
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  catkin_make --only-pkg-with-deps airsim_ros && \
  . devel/setup.sh && catkin_make --only-pkg-with-deps basic_dev && \
  . devel/setup.sh && catkin_make --only-pkg-with-deps imu_gps_odometry && \
  . devel/setup.sh && catkin_make --only-pkg-with-deps controller_test && \
  . devel/setup.sh && catkin_make --only-pkg-with-deps odometry

# 核心功能：指定容器启动时的默认执行程序。
ENTRYPOINT [ "/setup.bash" ]
