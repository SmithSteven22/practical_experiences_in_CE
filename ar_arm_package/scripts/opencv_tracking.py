#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
from moveit_commander import *
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf2_ros

# https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#
# http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29


if __name__ == '__main__':
    print("######## Start initialization ############")
    # First initialize moveit_commander and a rospy node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('opencv_tracking', anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

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

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tf_buffer.lookup_transform("link_base", "object", rospy.Time())

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("--> No object detected !")
            rate.sleep()
            continue

        target_pose = geometry_msgs.msg.Pose()

        target_pose.position.x = trans.transform.translation.x
        target_pose.position.y = trans.transform.translation.y
        target_pose.position.z = trans.transform.translation.z
        target_pose.orientation.x = trans.transform.rotation.x
        target_pose.orientation.y = trans.transform.rotation.y
        target_pose.orientation.z = trans.transform.rotation.z
        target_pose.orientation.w = trans.transform.rotation.w

        rospy.loginfo("Target pose {}".format(target_pose))

        # Cartesian Paths
        waypoints = []
        #waypoints.append(self.move_group.get_current_pose())
        waypoints.append(target_pose)


        """ Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints. Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory. """
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, # waypoints to follow
                                                                            0.01, # eef_step
                                                                            0.0, # jump_threshold
                                                                            True) # avoid_collisions
        print("fraction: ", fraction)
        move_group.execute(plan, wait=True) 
        
        rate.sleep()
                    




