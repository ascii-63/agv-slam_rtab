#include "agv-slam_rtabmap/agv-slam_rtabmap.h"

AGV_SLAM_RTABMap::AGV_SLAM_RTABMap(const ros::NodeHandle &_nh, const ros::NodeHandle &_nh_private)
    : nh_(_nh), nh_private_(_nh_private), tfListener(tfBuffer)
{
    // Initialize the publisher
    pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("rtabmap_pose", 10);

    // Create a timer to call the callback function at 10 Hz (100 ms intervals)
    timer_ = nh_.createTimer(ros::Duration(0.1), &AGV_SLAM_RTABMap::posePubCallback, this);
}

AGV_SLAM_RTABMap::~AGV_SLAM_RTABMap() {}

void AGV_SLAM_RTABMap::posePubCallback(const ros::TimerEvent &)
{
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        // Lookup the transform from 'odom' to 'robot_footprint'
        transformStamped = tfBuffer.lookupTransform("odom", "robot_footprint", ros::Time(0), ros::Duration(1.0));
        // transformStamped = tfBuffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(1.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("Could not get transform: %s", ex.what());
        return;
    }

    // Create a PoseStamped message
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = transformStamped.header.stamp;
    pose_msg.header.frame_id = "odom"; // Pose is in odom frame

    // Set position
    pose_msg.pose.position.x = transformStamped.transform.translation.x;
    pose_msg.pose.position.y = transformStamped.transform.translation.y;
    pose_msg.pose.position.z = transformStamped.transform.translation.z;

    // Set orientation
    pose_msg.pose.orientation = transformStamped.transform.rotation;

    // Convert quaternion to Euler angles to extract yaw
    tf2::Quaternion quat(
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // Publish the pose
    pose_pub.publish(pose_msg);

    ROS_INFO("Local Pose [odom -> footprint]: (%.5f, %.5f, %.5f)",
             pose_msg.pose.position.x,
             pose_msg.pose.position.y,
             yaw);
}
