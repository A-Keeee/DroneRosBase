#ifndef _IMU_GPS_FUSION_CPP_
#define _IMU_GPS_FUSION_CPP_

#include "imu_gps_fusion.hpp"

int main(int argc, char** argv)
{

    ros::init(argc, argv, "imu_gps_fusion"); // 初始化ros 节点，命名为 imu_gps_fusion
    ros::NodeHandle n; // 创建node控制句柄
    ImuGpsFusion go(&n);
    return 0;
}

ImuGpsFusion::ImuGpsFusion(ros::NodeHandle *nh)
{  
    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    odom_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/debug/pose_gt", 1, std::bind(&ImuGpsFusion::pose_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    gps_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/gps", 1, std::bind(&ImuGpsFusion::gps_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    imu_suber = nh->subscribe<sensor_msgs::Imu>("airsim_node/drone_1/imu/imu", 1, std::bind(&ImuGpsFusion::imu_cb, this, std::placeholders::_1));//imu数据

    ros::spin();
}

ImuGpsFusion::~ImuGpsFusion()
{
}

void ImuGpsFusion::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    ROS_INFO("Get pose data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void ImuGpsFusion::gps_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
    ROS_INFO("Get gps data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
        eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void ImuGpsFusion::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    ROS_INFO("Get imu data. time: %f", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9);
}


#endif