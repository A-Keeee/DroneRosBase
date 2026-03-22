import sys

readme_content = """# ORB-SLAM3

### V1.0, 2021年12月22日
**作者:** Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, [José M. M. Montiel](http://webdiis.unizar.es/~josemari/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/).

[更新日志(Changelog)](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Changelog.md) 描述了各个版本的特性。

ORB-SLAM3 是第一个能够使用**单目、双目和 RGB-D**相机（支持**针孔和鱼眼**镜头模型）进行**视觉、视觉惯性以及多地图 SLAM**的实时 SLAM 库。在所有传感器配置下，ORB-SLAM3 都与文献中现有的最佳系统一样鲁棒，并且精度显著提高。

我们提供了使用双目或单目（带或不带 IMU）在 [EuRoC 数据集](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) 上运行 ORB-SLAM3 的示例，以及使用鱼眼双目或单目（带或不带 IMU）在 [TUM-VI 数据集](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) 上运行的示例。可以在 [ORB-SLAM3 频道](https://www.youtube.com/channel/UCXVt-kXG6T95Z4tVaYlU80Q) 查看部分示例运行的演示视频。

本软件基于 [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) 开发，由 [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) 和 [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)) 开发。

<a href="https://youtu.be/HyLNq-98LRo" target="_blank"><img src="https://img.youtube.com/vi/HyLNq-98LRo/0.jpg" 
alt="ORB-SLAM3" width="240" height="180" border="10" /></a>

### 相关发表论文:

[ORB-SLAM3] Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel and Juan D. Tardós, **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**, *IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021*. **[PDF](https://arxiv.org/abs/2007.11898)**.

[IMU-Initialization] Carlos Campos, J. M. M. Montiel and Juan D. Tardós, **Inertial-Only Optimization for Visual-Inertial Initialization**, *ICRA 2020*. **[PDF](https://arxiv.org/pdf/2003.05766.pdf)**

[ORBSLAM-Atlas] Richard Elvira, J. M. M. Montiel and Juan D. Tardós, **ORBSLAM-Atlas: a robust and accurate multi-map system**, *IROS 2019*. **[PDF](https://arxiv.org/pdf/1908.11585.pdf)**.

[ORBSLAM-VI] Raúl Mur-Artal, and Juan D. Tardós, **Visual-inertial monocular SLAM with map reuse**, IEEE Robotics and Automation Letters, vol. 2 no. 2, pp. 796-803, 2017. **[PDF](https://arxiv.org/pdf/1610.05949.pdf)**. 

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://arxiv.org/pdf/1610.06475.pdf)**.

[Monocular] Raúl Mur-Artal, José M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](https://arxiv.org/pdf/1502.00956.pdf)**.

[DBoW2 Place Recognition] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp. 1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. 许可证 (License)

ORB-SLAM3 遵循 [GPLv3 许可证](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE) 发布。关于所有代码/库依赖项（及相关许可证）的列表，请参阅 [Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md)。

若需将 ORB-SLAM3 的闭源版本用于商业目的，请联系作者：orbslam (at) unizar (dot) es。

如果在学术工作中使用 ORB-SLAM3，请引用：
  
    @article{ORBSLAM3_TRO,
      title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
               and Multi-Map {SLAM}},
      author={Campos, Carlos AND Elvira, Richard AND G\´omez, Juan J. AND Montiel, 
              Jos\'e M. M. AND Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics}, 
      volume={37},
      number={6},
      pages={1874-1890},
      year={2021}
     }

# 2. 依赖项 (Prerequisites)
我们已经在 **Ubuntu 16.04** 和 **18.04** 中测试了该库，但在其他平台上也应该很容易编译。一台性能强劲的计算机（例如 i7 处理器）将确保您获得实时的性能，并提供更稳定和准确的结果。

## C++11 或 C++0x 编译器
我们使用了 C++11 的新的线程和 chrono（时间）功能。

## Pangolin
我们使用 [Pangolin](https://github.com/stevenlovegrove/Pangolin) 进行可视化和用户界面构建。下载和安装说明可参阅：https://github.com/stevenlovegrove/Pangolin.

## OpenCV
我们使用 [OpenCV](http://opencv.org) 来处理图像和特征。下载和安装说明可参阅：http://opencv.org. **最低需要 3.0 版本。已在 OpenCV 3.2.0 和 4.4.0 版本下成功测试**。

## Eigen3
g2o 所需（见下方）。下载和安装说明可参阅：http://eigen.tuxfamily.org. **最低需要 3.1.0 版本**。

## DBoW2 和 g2o（包含在 Thirdparty 文件夹中）
我们使用了修改版的 [DBoW2](https://github.com/dorian3d/DBoW2) 库来执行位置识别，并使用 [g2o](https://github.com/RainerKuemmerle/g2o) 库来执行非线性优化。这两个修改后的库（遵循 BSD 协议）都已包含在 *Thirdparty* 文件夹中。

## Python
用于计算轨迹与真实值（Ground Truth）之间的对齐情况。**需要 Numpy 模块**。

* (win) http://www.python.org/downloads/windows
* (deb) `sudo apt install libpython2.7-dev`
* (mac) 预装于 osx 系统中

## ROS (可选)

我们提供了一些通过 ROS 处理单目、单目-惯性、双目、双目-惯性或 RGB-D 相机输入的示例。构建这些示例是可选的。已经在 Ubuntu 18.04 下的 ROS Melodic 环境进行了测试。

# 3. 编译 ORB-SLAM3 库和示例代码

克隆本仓库：
```
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
```

我们提供了一个脚本 `build.sh` 用来编译 *Thirdparty* 库以及 *ORB-SLAM3*。请确保您已经安装了所有所需的依赖项（参见第2节）。执行：
```
cd ORB_SLAM3
chmod +x build.sh
./build.sh
```

这将在 *lib* 文件夹中生成 **libORB_SLAM3.so** 并在 *Examples* 文件夹中生成可执行文件。

# 4. 使用您自己的相机运行 ORB-SLAM3

`Examples` 目录包含多个演示程序和校准文件，用于在英特尔 RealSense T265 和 D435i 相机的所有传感器配置下运行 ORB-SLAM3。使用您自己的相机的相关步骤如下：

1. 按照 `Calibration_Tutorial.pdf` 校准您的相机并编写您的校准文件 `your_camera.yaml`

2. 修改提供的示例程序之一以适合您的具体相机模型并进行编译

3. 通过 USB3 或适当的接口将相机连接至您的计算机

4. 运行 ORB-SLAM3。例如，对我们的 D435i 相机，我们将执行：

```
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i Vocabulary/ORBvoc.txt ./Examples/Stereo-Inertial/RealSense_D435i.yaml
```

# 5. EuRoC 示例
[EuRoC 数据集](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) 是由两个针孔相机和惯性传感器记录的。我们提供了一个示例脚本，允许一次性启动所有传感器配置的 EuRoC 序列。

1. 从 http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets 下载序列 (ASL 格式)

2. 打开项目根目录中的 "euroc_examples.sh" 脚本。将 **pathDatasetEuroc** 变量更改为指向解压后数据集路径的目录。

3. 执行以下脚本以处理具有所有传感器配置的所有序列：
```
./euroc_examples
```

## 评估
EuRoC 提供了每个序列在 IMU 机体参考系下的真实值。由于纯视觉运行报告的是以左相机为中心的轨迹，我们在 "evaluation" 文件夹中提供了将真实值向左相机参考系转换的工具。视觉-惯性轨迹则是直接使用数据集提供的真实值。

执行以下脚本以处理序列并计算 RMS ATE ：
```
./euroc_eval_examples
```

# 6. TUM-VI 示例
[TUM-VI 数据集](https://vision.in.tum.de/data/datasets/visual-inertial-dataset) 是由两个鱼眼相机和一个惯性传感器记录的。

1. 从 https://vision.in.tum.de/data/datasets/visual-inertial-dataset 下载序列并解压。

2. 打开项目根目录下的 "tum_vi_examples.sh" 脚本。将 **pathDatasetTUM_VI** 变量更改为指向已解压的数据集所在目录。

3. 执行以下脚本来处理适用于所有传感器配置的所有序列：
```
./tum_vi_examples
```

## 评估
在 TUM-VI 数据集中，真实值只存在于所有序列开始和结束所在的房间。因此，误差衡量的是序列末尾处的漂移。

执行以下脚本来处理序列并计算 RMS ATE：
```
./tum_vi_eval_examples
```

# 7. ROS 示例

### 编译单目、单目-惯性、双目、双目-惯性和 RGB-D 的 ROS 节点
已在 ROS Melodic 加上 ubuntu 18.04 进行测试。

1. 将包含 *Examples/ROS/ORB_SLAM3* 的路径添加到 `ROS_PACKAGE_PATH` 环境变量中。打开 `.bashrc` 文件：
  ```
  gedit ~/.bashrc
  ```
并在文件末尾添加以下行。请将 PATH 替换为您克隆的 ORB_SLAM3 所在的文件夹路径：

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM3/Examples/ROS
  ```
  
2. 执行 `build_ros.sh` 脚本：

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### 运行单目 (Monocular) 节点
对于来自话题 `/camera/image_raw` 的单目输入，请运行 ORB_SLAM3/Mono 节点。您需要提供词汇表文件和配置文件。具体过程请参阅前面的单目示例。

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

### 运行单目-惯性 (Monocular-Inertial) 节点
对于来自话题 `/camera/image_raw` 的单目输入和来自话题 `/imu` 的惯性输入，请运行 ORB_SLAM3/Mono_Inertial 节点。将可选的第三个参数设置为 true 将会对图像应用限制对比度自适应直方图均衡化 (CLAHE)（主要用于 TUM-VI 数据集）。

  ```
  rosrun ORB_SLAM3 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE [EQUALIZATION]
  ```

### 运行双目 (Stereo) 节点
对于来自话题 `/camera/left/image_raw` 和 `/camera/right/image_raw` 的双目输入，请运行 ORB_SLAM3/Stereo 节点。您需要提供词汇表文件和配置文件。对于针孔相机模型，如果**提供了矫正矩阵**（参见 Examples/Stereo/EuRoC.yaml 示例），节点将在线矫正图像，**否则必须预先进行矫正**。对于鱼眼相机模型，无需进行矫正，因为系统直接处理原始图像：

  ```
  rosrun ORB_SLAM3 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```

### 运行双目-惯性 (Stereo-Inertial) 节点
对于来自话题 `/camera/left/image_raw` 和 `/camera/right/image_raw` 的双目输入，以及来自话题 `/imu` 的惯性数据，运行 ORB_SLAM3/Stereo_Inertial 节点。您需要提供词汇表文件和配置文件。如与双目节点一样，如有需要还要传递是否在线校正等参数。

  ```
  rosrun ORB_SLAM3 Stereo_Inertial PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION [EQUALIZATION]
  ```
  
### 运行 RGB_D 节点
对于来自话题 `/camera/rgb/image_raw` 和 `/camera/depth_registered/image_raw` 的 RGB-D 输入，运行 ORB_SLAM3/RGBD 节点。您需要提供词汇表文件和配置文件。请参阅前面的 RGB-D 示例。

  ```
  rosrun ORB_SLAM3 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```

**运行 ROS 示例：** 下载来自 EuRoC 数据集的一个 rosbag (例如 V1_02_medium.bag) (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)。在终端打开 3 个标签页并在每个标签页中运行以下命令以模拟 Stereo-Inertial 配置：
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
  ```
  
在 ORB-SLAM3 加载完毕词袋之后，请在 rosbag 这一选项卡下按空格键开始播放。

**补充注意：** 对于来自 TUM-VI 数据集的 rosbags，由于分块大小问题，可能会导致无法正常播放。一个可能的解决方案是使用默认的分块大小重新封包，例如：
  ```
  rosrun rosbag fastrebag.py dataset-room1_512_16.bag dataset-room1_512_16_small_chunks.bag
  ```

# 8. 运行时间分析
`include/Config.h` 中的一个宏开关可以激活时间评估。我们需要取消注释 `#define REGISTER_TIMES` 行以便获取单次执行的平均耗时统计显示在终端并存储到文本文件中（`ExecTimeMean.txt`）。

# 9. 相机校准
您可以在 `Calibration_Tutorial.pdf` 里面找到视觉-惯性联合校准的详细教程和有效配置文件对应内容的介绍说明。
"""

with open("/home/ake/RMUA/ORB_SLAM3/README.md", "w") as f:
    f.write(readme_content)

print("Done")
