#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Initialize the ROS node
rospy.init_node('motion', anonymous=True)

# Initialize MoveIt! Commander
moveit_commander.roscpp_initialize(sys.argv)

# Instantiate a MoveGroupCommander object. This object is an interface to a planning group (group of joints).
move_group = moveit_commander.MoveGroupCommander("xarm7")


# Define a callback function to handle incoming Pose messages
def pose_callback(pose_msg):

    # Plan and execute a Cartesian path to the target pose
    waypoints = [move_group.get_current_pose().pose, pose_msg]
    fraction = 0.0
    while fraction < 1.0:
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints,   # List of waypoints
            0.01,        # Step size for interpolation
            0.0          # Jump threshold
        )

    # Execute the planned trajectory
    move_group.execute(plan, wait=False)

    # clear the trajectory after execution 
    move_group.clear_pose_targets()
   


# Subscribe to the Pose topic (adjust topic name and message type)
rospy.Subscriber("motion", Pose, pose_callback)

# Keep the script running
rospy.spin()

# Clean up
moveit_commander.roscpp_shutdown()
