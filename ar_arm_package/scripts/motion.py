#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
from moveit_commander import *
import moveit_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
import geometry_msgs.msg
from geometry_msgs.msg import Point, PoseStamped, Pose



if __name__ == '__main__':
    print("######## Start initialization ############")
    # First initialize moveit_commander and a rospy node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion', anonymous=True)

    # Instantiate a RobotCommander object. Provides information such as the robot’s kinematic model and the robot’s current joint states
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a MoveGroupCommander object. This object is an interface to a planning group (group of joints). 
    move_group = moveit_commander.MoveGroupCommander("xarm7")

    move_group.set_pose_reference_frame("link_base")
    move_group.allow_replanning(False)
    move_group.set_goal_position_tolerance(0.05)
    move_group.set_goal_orientation_tolerance(0.05)

    # Create a `DisplayTrajectory`_ ROS publisher which is used to display
    # trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path",
        moveit_msgs.msg.DisplayTrajectory,
        queue_size=20,
    )
    print("############# Initialization complete ################")


    def callback(data):
        print(data)

        waypoints = []
        waypoints.append(data)

        """ Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints. Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory. """
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, # waypoints to follow
                                                                        0.01, # eef_step
                                                                            0.0, # jump_threshold
                                                                            True) # avoid_collisions
        # print("fraction: ", fraction)

        # # Execute the plan in a non-blocking manner
        move_group.execute(plan, wait=False)

        # # clear the trajectory after execution
        move_group.clear_pose_targets()


    rospy.Subscriber('/motion', geometry_msgs.msg.Pose, callback)

    rospy.spin()
    print("##############  Program closed !!!  ################")            


   


