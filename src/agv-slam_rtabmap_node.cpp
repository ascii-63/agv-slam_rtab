#include <ros/ros.h>
#include "agv-slam_rtabmap.h"

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "slam_pose_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    AGV_SLAM_RTABMap agv_slam(nh, nh_private);

    ros::spin();

    return 0;
}
