#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from moveit_commander import *
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
        self.pub_ = rospy.Publisher('/motion', geometry_msgs.msg.Pose, queue_size=5)

        self.rate = rospy.Rate(10)

        print("############# Initialization complete ################")


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

            print(target_pose)

            self.pub_.publish(target_pose)
            self.rate.sleep()

        else:
            rospy.loginfo("######## No tag detected #########")


if __name__ == '__main__':
    try:
        print("######## Start initialization ############")
        rospy.init_node('artag_detector', anonymous=True)
        Ar_track()

        rospy.spin()

        print("##############  Program closed !!!  ################")
    except rospy.ROSInterruptException:
        pass
