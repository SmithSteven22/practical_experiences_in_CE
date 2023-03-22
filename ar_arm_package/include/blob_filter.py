#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import cv2
import time
from pathlib import Path

trackbar_value_1 = 46
trackbar_value_2 = 49
trackbar_value_3 = 187
trackbar_value_4 = 117
trackbar_value_5 = 181
trackbar_value_6 = 245

exp = 0
wb = 1


#----------------//------------------
def updateValue_lH(new_value):
    global trackbar_value_1
    trackbar_value_1 = new_value

def updateValue_lS(new_value):
    global trackbar_value_2
    trackbar_value_2 = new_value

def updateValue_lV(new_value):
    global trackbar_value_3
    trackbar_value_3 = new_value

def updateValue_hH(new_value):
    global trackbar_value_4
    trackbar_value_4 = new_value

def updateValue_hS(new_value):
    global trackbar_value_5
    trackbar_value_5 = new_value

def updateValue_hV(new_value):
    global trackbar_value_6
    trackbar_value_6 = new_value
#-------------//////////--------------------

def exposure(new_value):
    global exp
    exp = new_value
    camera.set(cv2.CAP_PROP_EXPOSURE, exp)          # Exposure

def wb_temp(new_value):
    global wb
    wb = new_value
    camera.set(cv2.CAP_PROP_WB_TEMPERATURE, wb)      # White balance temperature
#-----------///////////----------------------------



def get_values_from_file(path_of_text_file): #------------------------------------------------------------
    global trackbar_value_1, trackbar_value_2, trackbar_value_3
    global trackbar_value_4, trackbar_value_5, trackbar_value_6, exp, wb

    path = Path(path_of_text_file)

    if path.is_file():
        print(f' The file {path} exists')

        # Displaying the contents of the text file
        file = open(path_of_text_file, "r")
        content = file.read()

        # Remove square brackets from list
        # using str() + list slicing
        content = str(content)[1:-1]

        # Split a string into a list where each word is a list item
        content = content.split()

        
        # print("\nContent in file1.txt:\n", content)
        trackbar_value_1 = int(content[0])
        trackbar_value_2 = int(content[1])
        trackbar_value_3 = int(content[2])
        trackbar_value_4 = int(content[3])
        trackbar_value_5 = int(content[4])
        trackbar_value_6 = int(content[5])

        exp = int(content[6])
        wb = int(content[7])

        print("\nTrackbar values:", trackbar_value_1, trackbar_value_2, trackbar_value_3, trackbar_value_4,
        trackbar_value_5, trackbar_value_6, exp, wb)

        file.close()

    else:
        print(f'The file {path} does not exists')

    return 
#-------------------////////------------------


def main(): #-----------------------------------------------------------------------
    get_values_from_file('trackbar_defaults.txt')
   
    global camera
    camera = cv2.VideoCapture(0)
    
    # Create a window and name it 'Thresholding'
    cv2.namedWindow("Thresholding")

    # Attach a trackbar to the window named 'Tresholding'
    cv2.createTrackbar("trackbar_lH", "Thresholding", trackbar_value_1, 179, updateValue_lH)
    cv2.createTrackbar("trackbar_lS", "Thresholding", trackbar_value_2, 255, updateValue_lS)
    cv2.createTrackbar("trackbar_lV", "Thresholding", trackbar_value_3, 255, updateValue_lV)
    cv2.createTrackbar("trackbar_hH", "Thresholding", trackbar_value_4, 179, updateValue_hH)
    cv2.createTrackbar("trackbar_hS", "Thresholding", trackbar_value_5, 255, updateValue_hS)
    cv2.createTrackbar("trackbar_hV", "Thresholding", trackbar_value_6, 255, updateValue_hV)

    cv2.createTrackbar("EXPOSURE", "Thresholding", exp, 500, exposure)
    cv2.createTrackbar("WB_TEMP", "Thresholding", wb, 500, wb_temp)

    prev_frame_time = 0
    new_frame_time = 0

    # Setup SimpleBlobDetector_params
    blobparams = cv2.SimpleBlobDetector_Params()

    while True:
        # Read the image from the camera
        ret, frame = camera.read()
        # frame = frame[200:250]

        blobparams.filterByArea = True
        blobparams.filterByCircularity = False
        blobparams.filterByInertia = False
        blobparams.filterByConvexity = False
        blobparams.minArea = 150
        blobparams.maxArea = 1E5
        detector = cv2.SimpleBlobDetector_create(blobparams)


        if not ret:
            break
        new_frame_time = time.time()
        fps = 1/(new_frame_time-prev_frame_time)
        prev_frame_time = new_frame_time

        fps = str(fps)
        cv2.putText(frame, fps, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # You will need this later
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Colour detection limits
        lH = trackbar_value_1
        lS = trackbar_value_2
        lV = trackbar_value_3
        hH = trackbar_value_4
        hS = trackbar_value_5
        hV = trackbar_value_6

        # Colour detection limits
        lowerLimits = np.array([lH, lS, lV])
        upperLimits = np.array([hH, hS, hV])

        thresholded = cv2.inRange(frame, lowerLimits, upperLimits)
        thresholded = cv2.bitwise_not(thresholded)

        kernal = np.ones((5,5), "uint8")
        thresholded = cv2.dilate(thresholded, kernal)

        keypoints = detector.detect(thresholded)
        thresholded = cv2.drawKeypoints(thresholded, keypoints, None, (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        for i in keypoints:
            cv2.putText(frame, str(i.pt[0]) + " " + str(i.pt[1]), (int(i.pt[0]), int(i.pt[1])),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
        
        cv2.imshow("Original", frame)        
        cv2.imshow("Thresholded", thresholded)


        # Quit the program when "q" is pressed
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            List = [trackbar_value_1, trackbar_value_2, trackbar_value_3, trackbar_value_4,
            trackbar_value_5, trackbar_value_6, exp, wb]

            storage = np.array(List)
            storage = str(storage)

            myfile = open('trackbar_defaults.txt', 'w')
            #print(List)

            myfile.write(storage)
            myfile.close()
            break

    # When everything done, release the camera
    print("closing program")
    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()