
// https://ros-planning.github.io/moveit_tutorials/doc/move_group_interface/move_group_interface_tutorial.html
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "ar_arm_package/target_position.h"

geometry_msgs::Pose target;
// The mutex class is a synchronization primitive that can be used to protect shared data from being simultaneously accessed by multiple threads.
std::mutex mtx;
using namespace std;

void Callback(ar_arm_package::target_position rcv)
{
    
    ROS_INFO_STREAM("Received pose: " << rcv);
    mtx.lock();
    // std::cout << "mutex locked in callback!" << std::endl;

    // target.orientation.w = 1;
    target.orientation.w = rcv.target_pose.pose.orientation.w;
    // target.orientation.x = rcv.target_pose.pose.orientation.x;
    // target.orientation.y = rcv.target_pose.pose.orientation.y;
    // target.orientation.z = rcv.target_pose.pose.orientation.z;
    target.position.x = rcv.target_pose.pose.position.x;
    target.position.y = rcv.target_pose.pose.position.y;
    target.position.z = rcv.target_pose.pose.position.z;
    mtx.unlock();

    // std::cout << "mutex unlocked in callback!" << std::endl;
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
    static const std::string PLANNING_GROUP = "xarm7";
    ros::Subscriber getpose_subscriber = n.subscribe("/xarm7/pose", 10, Callback);

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    // We will use the :planning_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group_interface.setEndEffectorLink("link_eef");
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    bool success = false;

    target.orientation.w = 1;
    target.orientation.x = 0;
    target.orientation.y = 0;
    target.orientation.z = 0;
    target.position.x = 0.596;
    target.position.y = -0.088;
    target.position.z = 0.071;


    while (ros::ok())
    {
        mtx.lock();
        // std::cout << "mutex locked!" << std::endl;
        move_group_interface.setPoseTarget(target);
        // success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        // ROS_INFO_STREAM("Moving........");
        move_group_interface.asyncMove();
        mtx.unlock();
        ros::Duration(3).sleep();
        // std::cout << "mutex unlocked!" << std::endl;
    }

    ros::waitForShutdown();
    // ros::shutdown();
    return 0;
}
