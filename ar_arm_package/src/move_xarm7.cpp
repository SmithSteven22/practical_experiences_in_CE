// https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "ar_arm_package/target_position.h"


using namespace std;



void Callback(ar_arm_package::target_position rcv){
// void Callback(const geometry_msgs::PoseStamped &rcv){

    // Planning group
    moveit::planning_interface::MoveGroupInterface move_group_interface_arm("xarm7");

    // geometry_msgs::PoseStamped rcv;
    ROS_INFO_STREAM("Received pose: " << rcv);


    // if (!rcv.empty()){
        // geometry_msgs::PoseStamped current_pose;
        // current_pose = move_group_interface_arm.getCurrentPose("link_eef");

        geometry_msgs::Pose target;

        target.orientation = rcv.target_pose.pose.orientation;
        target.position.x = rcv.target_pose.pose.position.x;
        target.position.y = rcv.target_pose.pose.position.y;
        target.position.z = rcv.target_pose.pose.position.z;
        // move_group_interface_arm.setPoseTarget(target);

        ROS_INFO_STREAM("Moving........");
        move_group_interface_arm.move();
    
    
    // } else{
    //     move_group_interface_arm.setNamedTarget("home");

    //     ROS_INFO_STREAM("Moving to home position...");
    //     move_group_interface_arm.move();

    // } 

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_xarm7");
    ros::NodeHandle n;

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber getpose_subscriber = n.subscribe("/xarm7/pose", 10, Callback);

    ros::waitForShutdown();
    // ros::shutdown();
    return 0;

}
