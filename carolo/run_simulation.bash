#!/bin/bash

#path of directory in which executed script is located
export DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source /usr/share/gazebo/setup.sh
source /opt/ros/melodic/setup.bash

#http://whatthecommit.com/

cd "$DIR"
cd ..
cd ..
cd ..

#echo $PWD

source ./devel/setup.bash #source /opt/ros/<distro>/setup.bash has to be commented out in the .bashrc file (otherwise the env variable will be always overwritten)

catkin_make clean

#build ROS Packages in workspace (e.g. to compile plugins)
catkin_make

cd "$DIR"

killall gzserver #kill all gazebo processes
roslaunch carolo gazebo_urdf.launch   # carolo.launch #start Gazebo with ROS

#$SHELL

