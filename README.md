ALG_ROS2_PUB
====================================  

This is demostration codes for get image from v4l2 and pub to ros2 topic.

# Prerequisites
Linux (Ubuntu 20.04)
   * CMake 3.5 or newer
   * gcc version 9.4.0 (Ubuntu 9.4.0-1ubuntu1~20.04.2)
   * Optional : nvidia-jetpack(5.1.3), ROS2 (foxy)

# Quick Build Instructions
1.  `mkdir build`  
2.  `cd build`  
3.  `cmake -DWITH_ROS2=ON -DCMAKE_INSTALL_PREFIX=<install path> ..`  
4.  `make`  
5.  `make install`  

# Publish ROS2 Image (Ubuntu)

Run ROS2 Publish
------------------------------------ 
1.   add ROS libraries to PATH: : 
     `source /opt/ros/foxy/setup.bash`  
2.   start ROS pub:  
     `./alg_ros_pub_node /dev/video0 GBRG`  
     para1: video device    para2: GBRG/GRBG/RCCC

Subscribe to ROS2 Topic 
------------------------------------
ROS2 Topic Name : `/image_data/stream/ch_xx`  
(xx is channel id starting from 00)  
To display image of channel 0, use rviz2 :   
`topic=/image_data/stream/ch_00`  

# Support
contact : stronger.quan@ailiteam.com
