#!/bin/sh
 source /opt/ros/electric/setup.bash
 export ROS_ROOT=/opt/ros/electric/ros
 export PATH=$ROS_ROOT/bin:$PATH
 export PYTHONPATH=$ROS_ROOT/core/roslib/src:$PYTHONPATH
 export ROS_PACKAGE_PATH=~/cs7630_robotics:/opt/ros/electric/stacks:$ROS_PACKAGE_PATH
