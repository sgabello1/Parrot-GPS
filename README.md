Parrot GPS
==========
![](https://github.com/sgabello1/Parrot-GPS/blob/master/parrot-ar-drone-flight-recorder-1.jpg)

This code has been written by a team of students from Politecnico di Torino for the exam of Robotics.  
This is a set of ROS nodes that exploit the Fligth recorder Parrot GPS module for the navigation in an outdoor environment.

Requirements
=============

The code has been tested in the project FLy4SmartCity ODOMI (https://github.com/fly4smartcity/odomi)
Requires ardrone_autonomy GPS branch (https://github.com/AutonomyLab/ardrone_autonomy/tree/gps ). 

The usage is as follow:

 - roslaunch ardrone_autonomy ardrone_aggressive.launch
 - rosrun mission_planner mission_planner
 - rosrun gps_communication main_fucntion_ok
 - open the Fly 4 Smart City GUI on the web browser (source code in ODOMI repository)
 - select the target clicking on the map and the click "send path" button

