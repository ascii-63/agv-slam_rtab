// File: robot_pose_publisher.cpp

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "robot_pose_publisher");
    ros::NodeHandle nh;

    // Create a publisher for the robot's pose
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 10);

    // Create a tf2 buffer and listener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Set the rate at which to publish the pose
    ros::Rate rate(10.0); // 10 Hz

    while (ros::ok())
    {
        geometry_msgs::TransformStamped transformStamped;

        try
        {
            // Lookup the transform from 'odom' to 'robot_footprint'
            transformStamped = tfBuffer.lookupTransform("odom", "robot_footprint", ros::Time(0), ros::Duration(1.0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Could not get transform: %s", ex.what());
            ros::Duration(0.1).sleep();
            continue;
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

        // Publish the pose
        pose_pub.publish(pose_msg);

        // Sleep to maintain loop rate
        rate.sleep();
    }

    return 0;
}
