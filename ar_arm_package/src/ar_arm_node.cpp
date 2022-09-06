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
    ros::init(argc, argv, "ar_arm_node");
    ros::NodeHandle n;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber ar_subscriber = n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("ar_pose_marker", 10, ar_pose);

    moveit::planning_interface::MoveGroupInterface move_group_interface_arm("xarm7");
    moveit::planning_interface::MoveGroupInterface move_group_interface_gripper("xarm_gripper");

    // 1. Home position...............................................
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
    // bool success = (move_group_interface_arm.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    move_group_interface_arm.move();

       ros::Rate loop_rate(10);

    while (ros::ok())
    {

        // 2. Place the TCP (Tool Center Point, the tip of the robot) above the object
        moveit::planning_interface::MoveGroupInterface::Plan TCP_plan;
        geometry_msgs::PoseStamped current_pose;
        current_pose = move_group_interface_arm.getCurrentPose("link_eef");

        geometry_msgs::Pose target_pose1;

        target_pose1.orientation = current_pose.pose.orientation;
        target_pose1.position.x = x;
        target_pose1.position.y = y;
        target_pose1.position.z = z;
        move_group_interface_arm.setPoseTarget(target_pose1);

        // bool success = (move_group_interface_arm.plan(TCP_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");
        move_group_interface_arm.move();

        // 1. Home position...............................................
        // moveit::planning_interface::MoveGroupInterface::Plan home_plan;
        // move_group_interface_arm.setJointValueTarget(move_group_interface_arm.getNamedTargetValues("home"));
        // bool success = (move_group_interface_arm.plan(home_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        // move_group_interface_arm.move();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
