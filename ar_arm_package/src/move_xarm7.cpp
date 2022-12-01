// Service
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "ar_arm_package/target_position.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_xarm7");
    ros::NodeHandle n;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber targetpose_subscriber = n.subscribe("/xarm7/pose", 10, callback);
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm("xarm7");
    



    // 1. Home position...............................................
    move_group_interface_arm.setNamedTarget("home");

    ROS_INFO_NAMED("tutorial", "Visualizing plan 1");
    move_group_interface_arm.move();


    // 2. Place the TCP (Tool Center Point, the tip of the robot) above the object
    geometry_msgs::PoseStamped current_pose;
    current_pose = move_group_interface_arm.getCurrentPose("link_eef");

    geometry_msgs::Pose target_pose1;

    target_pose1.orientation = current_pose.pose.orientation;
    target_pose1.position.x = 0.3;
    target_pose1.position.y = -0.4;
    target_pose1.position.z = 0.4;
    move_group_interface_arm.setPoseTarget(target_pose1);

    ROS_INFO_NAMED("tutorial", "Visualizing plan 2");
    move_group_interface_arm.move();


    // 4. TCP close to the object.......................................
    target_pose1.position.z = target_pose1.position.z - 0.4;
    move_group_interface_arm.setPoseTarget(target_pose1);
    move_group_interface_arm.move();

    ROS_INFO_NAMED("tutorial", "Visualizing plan 4");
    move_group_interface_arm.move();



    // 6. Move the TCP above the object............................................. 
    target_pose1.position.z += 0.2;
    target_pose1.position.x += 0.1 ;
    target_pose1.position.y += 0.6 ;
    move_group_interface_arm.setPoseTarget(target_pose1);
    move_group_interface_arm.move();

    ROS_INFO_NAMED("tutorial", "Visualizing plan 6");
    move_group_interface_arm.move();



    // 7. Lower TCP above the object..................................
    target_pose1.position.z = target_pose1.position.z - 0.14;
    move_group_interface_arm.setPoseTarget(target_pose1);
    move_group_interface_arm.move();

    ROS_INFO_NAMED("tutorial", "Visualizing plan 7");
    move_group_interface_arm.move();


    ros::shutdown();
    return 0;

}