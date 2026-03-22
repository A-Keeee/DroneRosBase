// ==============================================================================
// ORB-SLAM3 Stereo-Inertial ROS Node with Full Publisher Support
// 发布话题：里程计、位姿、轨迹路径、点云、跟踪状态、地图变更、TF
// ==============================================================================

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>

#include <set>

#include "System.h"
#include "ImuTypes.h"
#include "MapPoint.h"

using namespace std;

// ======================== 辅助函数 ========================

// 将 ORB-SLAM3 的 MapPoint 列表转换为 PointCloud2 消息
sensor_msgs::PointCloud2 MapPointsToPointCloud2(
    const std::vector<ORB_SLAM3::MapPoint*>& map_points,
    const ros::Time& stamp,
    const std::string& frame_id)
{
    // 预先统计有效点数
    int valid_count = 0;
    for (auto* pMP : map_points) {
        if (pMP && !pMP->isBad())
            valid_count++;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.height = 1;
    cloud_msg.width = valid_count;
    cloud_msg.is_dense = true;
    cloud_msg.is_bigendian = false;

    // 定义 point fields: x, y, z
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(valid_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (auto* pMP : map_points) {
        if (!pMP || pMP->isBad())
            continue;
        Eigen::Vector3f pos = pMP->GetWorldPos();
        *iter_x = pos(0);
        *iter_y = pos(1);
        *iter_z = pos(2);
        ++iter_x; ++iter_y; ++iter_z;
    }

    return cloud_msg;
}

// ======================== IMU Grabber ========================

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

// ======================== Image Grabber (含所有 Publisher) ========================

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb,
                 const bool bRect, const bool bClahe,
                 ros::NodeHandle& nh)
        : mpSLAM(pSLAM), mpImuGb(pImuGb),
          do_rectify(bRect), mbClahe(bClahe)
    {
        // 初始化所有 Publisher
        pub_odom           = nh.advertise<nav_msgs::Odometry>("/orb_slam3/odom", 10);
        pub_pose           = nh.advertise<geometry_msgs::PoseStamped>("/orb_slam3/pose", 10);
        pub_path           = nh.advertise<nav_msgs::Path>("/orb_slam3/path", 10);
        pub_tracked_points = nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/tracked_points", 5);
        pub_all_points     = nh.advertise<sensor_msgs::PointCloud2>("/orb_slam3/all_points", 2);
        pub_tracking_state = nh.advertise<std_msgs::Int32>("/orb_slam3/tracking_state", 10);
        pub_map_changed    = nh.advertise<std_msgs::Bool>("/orb_slam3/map_changed", 10);

        // 初始化路径消息
        path_msg.header.frame_id = "map";
    }

    void GrabImageLeft(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageRight(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft, mBufMutexRight;

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    // --- ROS Publishers ---
    ros::Publisher pub_odom;
    ros::Publisher pub_pose;
    ros::Publisher pub_path;
    ros::Publisher pub_tracked_points;
    ros::Publisher pub_all_points;
    ros::Publisher pub_tracking_state;
    ros::Publisher pub_map_changed;
    tf::TransformBroadcaster tf_broadcaster;

    // 轨迹路径消息（持续累积）
    nav_msgs::Path path_msg;

    // 累积所有已跟踪到的地图点指针（用于全局点云发布）
    std::set<ORB_SLAM3::MapPoint*> mAccumulatedMapPoints;
    int mFrameCounter = 0;
    static const int FULL_CLOUD_PUBLISH_INTERVAL = 30; // 每30帧发布一次全局点云

private:
    // 发布一帧的所有数据
    void PublishFrame(const Sophus::SE3f& Tcw, const double timestamp);
};


// ======================== main ========================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_inertial_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    bool bEqual = false;
    if(argc < 4 || argc > 5)
    {
        cerr << endl << "Usage: rosrun orb_slam3_ros_wrapper stereo_inertial_node path_to_vocabulary path_to_settings do_rectify [do_equalize]" << endl;
        ros::shutdown();
        return 1;
    }

    std::string sbRect(argv[3]);
    if(argc==5)
    {
        std::string sbEqual(argv[4]);
        if(sbEqual == "true")
            bEqual = true;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_STEREO,true);

    ImuGrabber imugb;
    ImageGrabber igb(&SLAM, &imugb, sbRect == "true", bEqual, n);

    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    // Maximum delay, 5 seconds
    ros::Subscriber sub_imu = n.subscribe("/airsim_node/drone_1/imu/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img_left = n.subscribe("/airsim_node/drone_1/front_left/Scene", 100, &ImageGrabber::GrabImageLeft,&igb);
    ros::Subscriber sub_img_right = n.subscribe("/airsim_node/drone_1/front_right/Scene", 100, &ImageGrabber::GrabImageRight,&igb);

    std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);

    ros::spin();

    return 0;
}


// ======================== ImageGrabber 实现 ========================

void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu()
{
    const double maxTimeDiff = 0.01;
    while(1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            this->mBufMutexRight.lock();
            while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
            {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
            {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
            {
                continue;
            }
            if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();
            if(mbClahe)
            {
                mClahe->apply(imLeft,imLeft);
                mClahe->apply(imRight,imRight);
            }

            if(do_rectify)
            {
                cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
                cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
            }

            // ====== 核心调用：跟踪并获取位姿 ======
            Sophus::SE3f Tcw = mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas);

            // ====== 发布所有 ROS 话题 ======
            PublishFrame(Tcw, tImLeft);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

// ======================== 发布一帧的所有数据 ========================

void ImageGrabber::PublishFrame(const Sophus::SE3f& Tcw, const double timestamp)
{
    ros::Time ros_time(timestamp);

    // ------ 1. 获取跟踪状态 ------
    int tracking_state = mpSLAM->GetTrackingState();

    // 发布跟踪状态
    std_msgs::Int32 state_msg;
    state_msg.data = tracking_state;
    pub_tracking_state.publish(state_msg);

    // ------ 2. 检测地图变更（回环闭合/重定位） ------
    bool map_changed = mpSLAM->MapChanged();
    std_msgs::Bool map_changed_msg;
    map_changed_msg.data = map_changed;
    pub_map_changed.publish(map_changed_msg);

    // ------ 3. 位姿发布（仅在跟踪状态 OK 时发布） ------
    // tracking_state == 2 表示 Tracking::OK
    if (tracking_state == 2)
    {
        // Tcw 是相机到世界的变换，取逆获得世界坐标系中的相机位姿
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf qwc = Twc.unit_quaternion();

        // --- PoseStamped ---
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros_time;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.position.x = twc(0);
        pose_msg.pose.position.y = twc(1);
        pose_msg.pose.position.z = twc(2);
        pose_msg.pose.orientation.x = qwc.x();
        pose_msg.pose.orientation.y = qwc.y();
        pose_msg.pose.orientation.z = qwc.z();
        pose_msg.pose.orientation.w = qwc.w();
        pub_pose.publish(pose_msg);

        // --- Odometry ---
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros_time;
        odom_msg.header.frame_id = "map";
        odom_msg.child_frame_id = "camera_link";
        odom_msg.pose.pose = pose_msg.pose;
        // 协方差设置为未知 (-1)，ORB-SLAM3 不直接提供协方差
        odom_msg.pose.covariance[0] = -1;
        odom_msg.twist.covariance[0] = -1;
        pub_odom.publish(odom_msg);

        // --- TF: map -> camera_link ---
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(twc(0), twc(1), twc(2)));
        transform.setRotation(tf::Quaternion(qwc.x(), qwc.y(), qwc.z(), qwc.w()));
        tf_broadcaster.sendTransform(
            tf::StampedTransform(transform, ros_time, "map", "camera_link"));

        // --- Path 轨迹路径 ---
        path_msg.header.stamp = ros_time;
        path_msg.poses.push_back(pose_msg);
        pub_path.publish(path_msg);

        // ------ 4. 当前帧跟踪到的地图点（稀疏点云） ------
        std::vector<ORB_SLAM3::MapPoint*> tracked_points = mpSLAM->GetTrackedMapPoints();
        if (!tracked_points.empty())
        {
            sensor_msgs::PointCloud2 tracked_cloud = MapPointsToPointCloud2(
                tracked_points, ros_time, "map");
            pub_tracked_points.publish(tracked_cloud);
        }

        // ------ 5. 累积全局地图点并定期发布 ------
        // 将当前帧跟踪到的地图点加入累积集合
        for (auto* pMP : tracked_points) {
            if (pMP && !pMP->isBad())
                mAccumulatedMapPoints.insert(pMP);
        }

        mFrameCounter++;
        // 定期发布全局点云，或在地图变更时立即发布
        if (map_changed || (mFrameCounter % FULL_CLOUD_PUBLISH_INTERVAL == 0))
        {
            // 清理已被标记为 bad 的点
            std::vector<ORB_SLAM3::MapPoint*> all_valid_points;
            all_valid_points.reserve(mAccumulatedMapPoints.size());
            for (auto it = mAccumulatedMapPoints.begin(); it != mAccumulatedMapPoints.end(); )
            {
                if (!(*it) || (*it)->isBad())
                    it = mAccumulatedMapPoints.erase(it);
                else {
                    all_valid_points.push_back(*it);
                    ++it;
                }
            }

            if (!all_valid_points.empty())
            {
                sensor_msgs::PointCloud2 all_cloud = MapPointsToPointCloud2(
                    all_valid_points, ros_time, "map");
                pub_all_points.publish(all_cloud);
            }

            if (map_changed)
                ROS_INFO("Map changed! Published %lu global map points.", all_valid_points.size());
        }
    }
    else
    {
        // 跟踪状态异常时输出警告
        if (tracking_state == 3)
            ROS_WARN_THROTTLE(2.0, "ORB-SLAM3: RECENTLY_LOST");
        else if (tracking_state == 4)
            ROS_WARN_THROTTLE(2.0, "ORB-SLAM3: LOST - attempting relocalization...");
    }
}


// ======================== ImuGrabber 实现 ========================

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}
