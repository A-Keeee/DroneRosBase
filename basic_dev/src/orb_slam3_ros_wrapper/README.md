# ORB-SLAM3 ROS Wrapper

This is a modern, standalone ROS (Catkin) package wrapper for [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3). 
It is configured to run the **Stereo-Inertial (双目-惯性)** SLAM node out of the box.

## 1. 依赖 (Dependencies)

在编译或运行此节点之前，必须确保：
1. **ORB-SLAM3 核心库已编译**：你必须已经进入与此 ROS 包同级的 `ORB_SLAM3` 源码目录下运行了 `./build.sh`，生成了 `libORB_SLAM3.so` 等核心库。
2. **Pangolin 已安装**：如果 Pangolin 安装在了系统目录（如 `/usr/local/lib`），需要确保该目录在系统的动态链接库搜索路径中。
3. **ROS Noetic/Melodic** 及相关常用视觉包（如 `cv_bridge`, `image_transport`, `sensor_msgs`）。

## 2. 编译 (Build)

如果你所在的工作空间（如 `/basic_dev`）内包含了不支持 catkin_make 的纯 CMake 包（如 Pangolin 或 Sophus 原码），推荐使用 `catkin build` 或配置 `CATKIN_IGNORE` 让 catkin 忽略它们。

正常的编译流程：
```bash
cd /basic_dev
catkin_make --pkg orb_slam3_ros_wrapper
```

## 3. 运行 (Run)

### 3.1 环境变量配置

> **重要**：以下所有命令必须在**同一个终端**中依次执行。

```bash
# 1. 加载 ROS 基础环境
source /opt/ros/noetic/setup.bash

# 2. 加载 Pangolin 等本地库路径
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# 3. 加载工作空间（必须在 source ROS 之后执行）
cd /basic_dev
source devel/setup.bash
```

建议将以上命令添加到 `~/.bashrc` 以免每次手动执行：
```bash
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'source /basic_dev/devel/setup.bash' >> ~/.bashrc
```

### 3.2 确保 roscore 已运行

在**另一个终端**中运行：
```bash
roscore
```

### 3.3 启动 Stereo-Inertial 节点

本节点需要传入至少 **3个参数**：
1. **ORB_SLAM3 的词袋路径** (Vocabulary)
2. **相机 YAML 配置文件路径** (Settings)
3. **是否开启极线矫正** (`true`/`false`): 如果图像未经矫正，请设为 `true`；如果输入图像已经是极线矫正过的了，请设为 `false`。
4. **[可选] 是否开启直方图均衡化** (`true`/`false`): 提高对比度，多用于 TUM-VI 数据集。

**运行示例（使用 EuRoC 配置）：**

```bash
rosrun orb_slam3_ros_wrapper stereo_inertial_node \
    /basic_dev/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    /basic_dev/src/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml \
    false \
    false
```
*(注意：请将第二个参数的 YAML 配置文件替换为你当前实际使用的双目相机参数文件)*

### 3.4 使用 Launch 文件启动（推荐）

推荐使用 launch 文件一键启动节点 + RViz 可视化：

```bash
# 默认参数启动（使用 EuRoC 配置）
roslaunch orb_slam3_ros_wrapper slam.launch

# 自定义参数启动
roslaunch orb_slam3_ros_wrapper slam.launch \
    settings_file:=/path/to/your/camera.yaml \
    do_rectify:=true \
    do_equalize:=false

# 不启动 RViz
roslaunch orb_slam3_ros_wrapper slam.launch open_rviz:=false
```

**可配置参数：**

| 参数 | 默认值 | 说明 |
|---|---|---|
| `voc_file` | ORBvoc.txt | ORB 词袋文件路径 |
| `settings_file` | EuRoC.yaml | 相机 YAML 配置文件路径 |
| `do_rectify` | `false` | 是否开启极线矫正 |
| `do_equalize` | `false` | 是否开启直方图均衡化 |
| `open_rviz` | `true` | 是否同时打开 RViz |

## 4. 话题接口 (Topics)

### 4.1 订阅话题 (Subscribed Topics)

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/airsim_node/drone_1/imu/imu` | `sensor_msgs/Imu` | IMU 数据 |
| `/airsim_node/drone_1/front_left/Scene` | `sensor_msgs/Image` | 左目图像 |
| `/airsim_node/drone_1/front_right/Scene` | `sensor_msgs/Image` | 右目图像 |

### 4.2 发布话题 (Published Topics)

| 话题 | 消息类型 | 说明 |
|---|---|---|
| `/orb_slam3/odom` | `nav_msgs/Odometry` | 视觉惯性里程计，可直接对接 `move_base` 等导航栈 |
| `/orb_slam3/pose` | `geometry_msgs/PoseStamped` | 当前帧相机位姿（世界坐标系） |
| `/orb_slam3/path` | `nav_msgs/Path` | 历史轨迹路径（持续累积） |
| `/orb_slam3/tracked_points` | `sensor_msgs/PointCloud2` | 当前帧跟踪到的稀疏地图点 |
| `/orb_slam3/all_points` | `sensor_msgs/PointCloud2` | 全局累积点云（每30帧或地图变更时更新） |
| `/orb_slam3/tracking_state` | `std_msgs/Int32` | 跟踪状态码（见下表） |
| `/orb_slam3/map_changed` | `std_msgs/Bool` | 地图变更通知（回环闭合/全局BA） |

**TF**：发布 `map` → `camera_link` 坐标变换。

### 4.3 跟踪状态码 (Tracking State)

| 状态码 | 含义 | 说明 |
|---|---|---|
| -1 | `SYSTEM_NOT_READY` | 系统未就绪 |
| 0 | `NO_IMAGES_YET` | 尚未接收到图像 |
| 1 | `NOT_INITIALIZED` | 初始化中 |
| **2** | **`OK`** | 正常跟踪 |
| 3 | `RECENTLY_LOST` | 近期丢失，尝试恢复 |
| 4 | `LOST` | 跟踪丢失，触发重定位 |

## 5. 话题重映射 (Topic Remapping)

如果你的传感器或者 ROS bag 发布的话题名称不同，可在运行时做话题重映射：

```bash
rosrun orb_slam3_ros_wrapper stereo_inertial_node \
    /basic_dev/src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    /basic_dev/src/ORB_SLAM3/Examples/Stereo-Inertial/EuRoC.yaml \
    false \
    /airsim_node/drone_1/imu/imu:=/your/custom/imu_topic \
    /airsim_node/drone_1/front_left/Scene:=/your/custom/left_image \
    /airsim_node/drone_1/front_right/Scene:=/your/custom/right_image
```

## 6. RViz 可视化

启动 RViz 后，可添加以下显示类型以可视化 SLAM 输出：

```bash
rviz
```

- **Odometry** → 话题设为 `/orb_slam3/odom`
- **Path** → 话题设为 `/orb_slam3/path`
- **PointCloud2** → 话题设为 `/orb_slam3/tracked_points` 或 `/orb_slam3/all_points`
- **TF** → 可查看 `map` → `camera_link` 变换
