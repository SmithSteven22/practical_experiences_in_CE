#!/usr/bin/env python3

# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# http://wiki.ros.org/tf2/Tutorials/Adding%20a%20frame%20%28Python%29

import sys
import cv2
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
import geometry_msgs.msg
from geometry_msgs.msg import Point, PoseStamped, Pose
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf2_ros

class image_converter:
    
    def __init__(self):    
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :

            # Setup SimpleBlobDetector_params
            blobparams = cv2.SimpleBlobDetector_Params()

            blobparams.filterByArea = True
            blobparams.filterByCircularity = False
            blobparams.filterByInertia = False
            blobparams.filterByConvexity = False
            blobparams.minArea = 150
            blobparams.maxArea = 1E5

            detector = cv2.SimpleBlobDetector_create(blobparams)

            # Convert image from BRG to HSV
            img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Color detection limits
            hsvMin = np.array([31,48,38])
            hsvMax = np.array([88,255,205])

            # Apply HSV thresholds
            img_mask = cv2.inRange(img_hsv, hsvMin, hsvMax)
            img_mask = cv2.bitwise_not(img_mask)

            kernal = np.ones((5,5), "uint8")
            img_mask = cv2.dilate(img_mask, kernal)

            keypoints = detector.detect(img_mask)
            img_mask = cv2.drawKeypoints(img_mask, keypoints, None, (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            br = tf2_ros.TransformBroadcaster()
            blob_pose = geometry_msgs.msg.TransformStamped()

            for i, kp in enumerate(keypoints):
                if len(keypoints) == 1:
                    x = kp.pt[0]
                    y = kp.pt[1]
                    s = kp.size
                    # print(f"x = {x}, y = {y}, s = {s}")

                    # Find coordinates in camera frame
                    rows = float(cv_image.shape[0])
                    cols = float(cv_image.shape[1])
                    # print(f"rows = {rows}, cols = {cols}")
                    center_x = 0.5*cols
                    center_y = 0.5*rows

                    x = (kp.pt[0] - center_x)/(center_x)
                    y = (kp.pt[1] - center_y)/(center_y)
                    s = kp.size/cv_image.shape[1]

                    blob_pose.header.frame_id = "usb_cam"
                    blob_pose.child_frame_id = "object"
                    blob_pose.header.stamp = rospy.Time.now()
                    blob_pose.transform.translation.x = x
                    blob_pose.transform.translation.y = y
                    blob_pose.transform.translation.z = s
                    blob_pose.transform.rotation.x = 0.0
                    blob_pose.transform.rotation.y = 0.0 
                    blob_pose.transform.rotation.z = 0.0
                    blob_pose.transform.rotation.w = 1.0

                    br.sendTransform(blob_pose)
                    
        cv2.imshow("Image mask", img_mask)
        cv2.waitKey(3)


def main(args):
    print("############## Start initialization #########")

    ic = image_converter()
    rospy.init_node("opencv_detector", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down !!!")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

