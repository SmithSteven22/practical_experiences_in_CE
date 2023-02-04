
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "ar_arm_package/target_position.h"



class ArmSubscriber{
public:
	ArmSubscriber(ros::NodeHandle &n);
	void poseCallback(ar_arm_package::target_position rcv);

protected:
	const std::string _PLANNING_GROUP;
	moveit::planning_interface::MoveGroupInterface _move_group;
	moveit::planning_interface::PlanningSceneInterface _planning_scene_interface;
	const robot_state::JointModelGroup* _joint_model_group;
	moveit_visual_tools::MoveItVisualTools _visual_tools;
	ros::Subscriber _pose_sub;
};


ArmSubscriber::ArmSubscriber(ros::NodeHandle &n) : _PLANNING_GROUP("xarm7"),
	_move_group(_PLANNING_GROUP), _visual_tools("base_link"){

	_joint_model_group = _move_group.getCurrentState()->getJointModelGroup(_PLANNING_GROUP);
	_pose_sub = n.subscribe("/xarm7/pose", 1000, &ArmSubscriber::poseCallback, this);
	_visual_tools.deleteAllMarkers();
}

void ArmSubscriber::poseCallback(ar_arm_package::target_position rcv){
	ROS_INFO("Received pose");

	geometry_msgs::Pose target_pose;

	target_pose.position.x = rcv.target_pose.pose.position.x;
	target_pose.position.y = rcv.target_pose.pose.position.y;
	target_pose.position.z = rcv.target_pose.pose.position.z;
	target_pose.orientation.w = rcv.target_pose.pose.orientation.w;
	target_pose.orientation.x = rcv.target_pose.pose.orientation.x;
	target_pose.orientation.y = rcv.target_pose.pose.orientation.y;
	target_pose.orientation.z = rcv.target_pose.pose.orientation.z;

	_move_group.setPoseTarget(target_pose);

	moveit::planning_interface::MoveGroupInterface::Plan plan;

	ROS_INFO("Planning starting...");
	bool success = (_move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO("Planning: %s", success ? "successful" : "unsuccessful");

	_visual_tools.publishAxisLabeled(target_pose, "target_pose");
	_visual_tools.publishTrajectoryLine(plan.trajectory_, _joint_model_group);

	ROS_INFO("Executing...");
	if(success){
		_move_group.execute(plan);
		ROS_INFO("Plan executed");
	}
	else{
		ROS_INFO("No plan to execute");
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "armove");
	ros::NodeHandle node_handle;

	// Spin two threads, necessary or else planner gets stuck in callback
	ros::AsyncSpinner spinner(2);
	spinner.start();

	ArmSubscriber armSub(node_handle);

	ros::waitForShutdown();

	return 0;
}