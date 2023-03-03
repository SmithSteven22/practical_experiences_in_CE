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


class Ar_track:

    def __init__(self): 
        self.tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf_buffer)
        
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)

        # Instantiate a RobotCommander object. Provides information such as the robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander object. This object is an interface to a planning group (group of joints). 
        self.move_group = moveit_commander.MoveGroupCommander("xarm7")

        self.move_group.set_pose_reference_frame("link_base")
        self.move_group.allow_replanning(False)
        self.move_group.set_goal_position_tolerance(0.05)
        self.move_group.set_goal_orientation_tolerance(0.05)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        print("############# Initialization complete ################")



    # # Cartesian Paths
    def move_arm(self, target_pose):  # ----------------------------------------------------------------------------------------------------

        waypoints = []
        #waypoints.append(self.move_group.get_current_pose())
        waypoints.append(target_pose)


        """ Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the poses specified as waypoints. Configurations are computed for every eef_step meters; The jump_threshold specifies the maximum distance in configuration space between consecutive points in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed, the actual RobotTrajectory. """
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, # waypoints to follow
                                                                           0.01, # eef_step
                                                                            0.0, # jump_threshold
                                                                            True) # avoid_collisions
        print("fraction: ", fraction)
        self.move_group.execute(plan, wait=True)
        # self.move_group.stop()
        # self.move_group.clear_pose_targets() 


    def callback(self, msg):  # ---------------------------------------------------------------------------------------

        trans = geometry_msgs.msg.TransformStamped()

        if msg.markers:
            # print(msg)
            while True:
                try:
                    trans = self.tf_buffer.lookup_transform("link_base", "artag", rospy.Time(0), rospy.Duration(1))
                    break

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
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

            self.move_arm(target_pose)

            # self.move_group.set_pose_target(target_pose)
            # self.move_group.go(wait=True)

        else:
            rospy.loginfo("######## No tag detected #########")



if __name__ == '__main__':
    try:
        print("######## Start initialization ############")
        # First initialize moveit_commander and a rospy node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot', anonymous=True)
        Ar_track()

        rospy.spin()

        print("##############  Program closed !!!  ################")
    except rospy.ROSInterruptException:
        pass
