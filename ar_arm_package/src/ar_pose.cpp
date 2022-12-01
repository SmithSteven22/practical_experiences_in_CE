#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include "geometry_msgs/Twist.h"
#include <iostream>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "tf/transform_datatypes.h"

using namespace std;

double x, y, z;



void ar_pose(ar_track_alvar_msgs::AlvarMarkers ar_msg)
{
    if (ar_msg.markers.size() > 0 and ar_msg.markers.size() <= 1)
    {
        // we transform the quartenion to rpy
        x = ar_msg.markers[0].pose.pose.position.x;
        y = ar_msg.markers[0].pose.pose.position.y;
        z = ar_msg.markers[0].pose.pose.position.z;
    }

    ROS_INFO("x=%1.2f  y=%1.2f  z=%1.2f", x, y, z);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_pose");
    ros::NodeHandle n;

    ros::Subscriber ar_subscriber = n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 10, ar_pose);

    ros::Rate loop_rate(10);

    ros::spin();
    return 0;
}