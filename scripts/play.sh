#!/bin/bash

cd $(rospack find robot_tutorial_1)/urdf
rosrun xacro xacro wheely_boi.xacro > wb.urdf

x-terminal-emulator -e roslaunch robot_tutorial_1 play.launch

rosrun gazebo_ros spawn_model -file wb.urdf -urdf -model wheely_boi

rosrun robot_tutorial_1 keyin

cd $(rospack find robot_tutorial_1)/urdf
rm wb.urdf

