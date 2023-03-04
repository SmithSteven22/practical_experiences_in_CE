<a id="top"></a>
# Task description

The objective is to assess different camera-based machine vision approaches for guiding a manipulator end-effector (eef) during a pick and place task

Tasks:
1) Find different ROS software packages that can be integrated with a camera for object recognition
2) Develop a software pipeline that takes the coordinates of an object (based on machine vision) and guides the eef to pick the object
  - To develop the pipeline, feel free to use ar_track_alvar for object recognition and MoveIt with xArm7 manipulator. 
3) Assess the quality, stability, speed, ease-of-integration for different machine-vision solutions. Make sure to highlight any bottlenecks and technical limitations in the final report. 

Deliverable results:
1) Video demonstrations of vision-based eef guidance (at minimum: physical camera, simulated robot)
2) Code and documentation is available on GitHub
3) Final written report that describes the outputs of each task



# Dependencies 
- [xArm](https://github.com/xArm-Developer/xarm_ros)
- [Usb camera](http://wiki.ros.org/usb_cam)
- [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)
- [find_object_2d](http://wiki.ros.org/find_object_2d)

# Artag tracking
```
roslaunch ar_arm_package usb_cam_ar_tracking.launch
rosrun ar_arm_package ar_tracking.py
```
![Artag tracking](https://github.com/SmithSteven22/practical_experiences_in_CE/blob/ims_project/ar_control.gif)

