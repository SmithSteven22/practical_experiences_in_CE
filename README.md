<a id="top"></a>
# Task description

Create a keyboard-controlled robot solution. When the user presses a certain key, the robot starts moving in a certain way. Please program the following behvior:
- when "O" is pressed, the robot moves on a circular path;
- when "S" is pressed, the robot moves on an S-shaped path;
- when "8" is pressed, the robot moves on an 8-shaped path;
- when either "Z" or "N" is pressed the robot moves on a zigzag path;
- when "T" is pressed, the robot stops.


Solution

Add "task_one" package to your workspace, source and catkin build.
Use the following commande to execute the robotont_driver node:
- roslaunch robotont_driver fake_driver.launch

  *To launch the vertual robot "Robotont" in rviz*
- rosrun task_one robotont_driver
