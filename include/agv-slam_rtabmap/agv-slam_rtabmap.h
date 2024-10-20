#ifndef AGV_SLAM_RTABMAP_H
#define AGV_SLAM_RTABMAP_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class AGV_SLAM_RTABMap
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Publisher pose_pub;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    ros::Timer timer_;

    /**
     * Callback function for publishing the robot's pose.
     *
     * This function looks up the transform from 'odom' to 'robot_footprint' using tfBuffer,
     * creates a PoseStamped message with the transform data, and publishes it using pose_pub.
     * If the transform lookup fails, it logs a warning and returns without publishing.
     */
    void posePubCallback(const ros::TimerEvent &);

public:
    AGV_SLAM_RTABMap(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
    virtual ~AGV_SLAM_RTABMap();
};

#endif
