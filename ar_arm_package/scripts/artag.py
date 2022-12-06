#!/usr/bin/env python3

import rospy
import math
import tf
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
import pdb
from ar_arm_package.msg import target_position


msg_target = target_position()


def callback(data, listener):

    try:

        # This function returns two lists. The first is the (x, y, z) linear transformation of the child frame relative to the parent, and the
        # second is the (x, y, z, w) quaternion required to rotate from the parent orientation to the child orientation.
        trans, rot = listener.lookupTransform('/world', '/ar_marker_5', rospy.Time())

        msg_target.target_pose.pose.position.x = trans[0]
        msg_target.target_pose.pose.position.y = trans[1]
        msg_target.target_pose.pose.position.z = trans[2]
        msg_target.target_pose.pose.orientation.x = rot[0]
        msg_target.target_pose.pose.orientation.y = rot[1]
        msg_target.target_pose.pose.orientation.z = rot[2]
        msg_target.target_pose.pose.orientation.w = rot[3]
        msg_target.target_pose.header.frame_id = "world"
        msg_target.target_pose.header.stamp = rospy.Time.now()

        print("trans=", trans, "rot", rot)

    except (tf.LookupException, tf.ConnectivityException,   tf.ExtrapolationException):
        pass


###########################################################################
if __name__ == '__main__':
    try:
        rospy.init_node('artag', anonymous=True)

        listener = tf.TransformListener()

        ar_subscriber = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback, listener)
        ar_target_publisher = rospy.Publisher('/xarm7/pose', target_position, queue_size=1)

        rate = rospy.Rate(60.0)

        while not rospy.is_shutdown():

            ar_target_publisher.publish(msg_target)
            # tag_status.publish(single_tag_on_screen)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
