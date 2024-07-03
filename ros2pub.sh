#!/bin/bash


source /opt/ros/foxy/setup.bash
export LD_LIBRARY_PATH=/opt/ros/foxy/lib/:$LD_LIBRARY_PATH

alg_ros_pub_node /dev/video0 GBRG &
alg_ros_pub_node /dev/video1 GRBG &
alg_ros_pub_node /dev/video2 GRBG &
alg_ros_pub_node /dev/video3 GBRG &
alg_ros_pub_node /dev/video4 RCCC


