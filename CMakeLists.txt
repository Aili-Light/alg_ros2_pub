cmake_minimum_required(VERSION 3.5)
project(alg_ros_pub)


link_directories(thirdparty/lib)
link_directories(/usr/local/cuda/lib64)
set(CMAKE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/release/linux)
set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "\${ORIGIN}/../lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# if (WITH_ROS)
#   message("-- Build with ROS")
#   add_definitions(-DWITH_ROS)
#   find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     rospy
#     std_msgs
#     sensor_msgs
#   )

#   include_directories(
#     ${catkin_INCLUDE_DIRS}
#     src
#     thirdparty/include
#     include
#     /usr/local/cuda/include
#   )

#   add_executable(${PROJECT_NAME}_node 
#     src/alg_ros_pub_node.cpp
#     src/alg_rospub.cpp
#   )

#   target_link_libraries(${PROJECT_NAME}_node
#     ${catkin_LIBRARIES}
#     libalg_cv_cvtColor_cuda.so
#     libalg_cv_cvtColor.so
#     libalg_cv_IspPipe.so
#   )
# endif ()

if (WITH_ROS2)
  message("-- Build with ROS2")
  add_definitions(-DWITH_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcl_interfaces REQUIRED)
  find_package(sensor_msgs REQUIRED)

  include_directories(
      ${rclcpp_INCLUDE_DIRS}
      ${rcl_interfaces_INCLUDE_DIRS}
      ${sensor_msgs_INCLUDE_DIRS}
      src
      thirdparty/include
      include
      /usr/local/cuda/include
  )

  add_executable(${PROJECT_NAME}_node
      src/alg_ros_pub_node.cpp
      src/alg_ros2pub.cpp
  )

  target_link_libraries(${PROJECT_NAME}_node
      libalg_cv_cvtColor_cuda.so
      libalg_cv_cvtColor.so
      libalg_cv_IspPipe.so
      pthread
      anl
      ${ament_LIBRARIES}
      ${rclcpp_LIBRARIES}
      ${sensor_msgs_LIBRARIES}
  )

  set_target_properties(${PROJECT_NAME}_node PROPERTIES LINKER_LANGUAGE CXX)
  install (TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CMAKE_INSTALL_DIR}/bin)
  FILE(GLOB ALG_CVT_LIB "${CMAKE_SOURCE_DIR}/thirdparty/lib/libalg_cv*")
  install (FILES ${ALG_CVT_LIB} DESTINATION ${CMAKE_INSTALL_DIR}/lib)
  install (FILES ros2pub.sh DESTINATION ${CMAKE_INSTALL_DIR}/bin)


endif ()







