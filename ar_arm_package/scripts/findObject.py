#!/usr/bin/env python3

import sys
import tf2_ros
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Point, PoseStamped, Pose
from find_object_2d.msg import ObjectsStamped
import std_msgs.msg
from PySide2.QtGui import QTransform
from PySide2.QtCore import QPointF
from functools import cmp_to_key
import math


class ObjectPoseDetector:
    
    def __init__(self): 

        self.objects_sub = rospy.Subscriber('/objectsStamped', ObjectsStamped, self.cornersDetectedCallback)

        self.pub_ = rospy.Publisher('/motion', Pose, queue_size=5)

        self.rate = rospy.Rate(10)

        print("############# Initialization complete ################")


    # Subscriber callback
    def cornersDetectedCallback(self, msg):

        # Creation of a Corners object to publish the info
        coordinates = []

        object_pose = Pose()

        # Data saving in temporal variables
        data = msg.objects.data
        if len(data) > 0:
            for i in range(0, len(data), 12): # For to extract data given by the find object 2d topic
                # Get data
                id = int(data[i])
                objectWidth = data[i+1]
                objectHeight = data[i+2]

                # Find corners Qt
                # QTransform initialization with the homography matrix
                qtHomography = QTransform(data[i+3], data[i+4], data[i+5],
                                        data[i+6], data[i+7], data[i+8],
                                        data[i+9], data[i+10], data[i+11])

                # Map the template coordinates to the current frame with the shape of the template and homography
                qtTopLeft = qtHomography.map(QPointF(0,0)) # Top left coordinates
                qtTopRight = qtHomography.map(QPointF(objectWidth,0)) # Top right coordinates
                qtBottomLeft = qtHomography.map(QPointF(0,objectHeight)) # Bottom left coordinates
                qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight)) # Bottom right coordinates
                qCenter = qtHomography.map(QPointF(objectWidth/2,objectHeight/2)) # Centroid Coordinates

                # First Column is X coordinate, second is Y.
                points = [qtTopLeft, qtTopRight, qtBottomLeft, qtBottomRight]

                # Sort the points with respect to X inside the vector of points
                points.sort(key=cmp_to_key(sorter))

                # Assign coordinates to the publishing object
                if points[0].y() > points[1].y():
                   BottomLeftX = points[0].x()
                   BottomLeftY = points[0].y()
                   TopLeftX = points[1].x()
                   TopLeftY = points[1].y()
                else:
                   BottomLeftX = points[1].x()
                   BottomLeftY = points[1].y()
                   TopLeftX = points[0].x()
                   TopLeftY = points[0].y()

                # Assign the bottom right and top right corners
                if points[2].y() > points[3].y():
                   BottomRightX = points[2].x()
                   BottomRightY = points[2].y()
                   TopRightX = points[3].x()
                   TopRightY = points[3].y()
                else:
                   BottomRightX = points[3].x()
                   BottomRightY = points[3].y()
                   TopRightX = points[2].x()
                   TopRightY = points[2].y()

                # Assign centroid values
                CenterX = qCenter.x()
                CenterY = qCenter.y()

                # Uncomment these lines to print the results on console
                """
                print("Qt corners at ({},{}) ({},{}) ({},{}) ({},{})".format(
                   TopLeftX,TopLeftY,
                   TopRightX,TopRightY,
                   BottomLeftX,BottomLeftY,
                   BottomRightX,BottomRightY
                ))
                """

        # Assign zero to all the members of the publishing object
        else:
           TopLeftX = 0
           TopLeftY = 0
           TopRightX = 0
           TopRightY = 0
           BottomLeftX = 0
           BottomLeftY = 0
           BottomRightX = 0
           BottomRightY = 0
           CenterX = 0
           CenterY = 0


        W = abs(((TopLeftX+BottomLeftX)/2) - ((TopRightX+BottomRightX)/2))
        H = abs(((TopLeftY+BottomLeftY)/2) - ((TopRightY+BottomRightY)/2))

        area = H*W
        diameter = math.sqrt((4*area)/math.pi)

        object_pose.position.x = CenterX
        object_pose.position.y = CenterY
        object_pose.position.z = diameter
        object_pose.orientation.x = 0.0
        object_pose.orientation.y = 0.0
        object_pose.orientation.z = 0.0
        object_pose.orientation.w = 1.0
      
        # print(f"Center x: {round(CenterX, 2)}, Center y: {round(CenterY,2)}, area: {round(area,2)}, d: {diameter}")
        print(object_pose)
        self.pub_.publish(object_pose)


# Sorter function for sorting points based on X coordinate
def sorter(point1, point2):
    if point1.x() < point2.x():
        return -1
    elif point1.x() > point2.x():
        return 1
    else:
        return 0
    
    
if __name__ == '__main__':
    try:
        print("######## Start initialization ############")
        rospy.init_node('findObject', anonymous=True)
        ObjectPoseDetector()

        rospy.spin()

        print("##############  Program closed !!!  ################")
    except rospy.ROSInterruptException:
        pass