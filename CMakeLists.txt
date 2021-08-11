cmake_minimum_required(VERSION 3.0.2)
project(tray_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  tf
  sensor_msgs
  image_transport
  pcl_conversions
  pcl_ros
  darknet_ros_msgs
)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime cv_bridge rospy roscpp std_msgs geometry_msgs tf sensor_msgs image_transport pcl_conversions pcl_ros
)

LIST(APPEND LINK_LIBS
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(tray_detector
  src/tray_detector.cpp
  ${SOURCE_FILES}
)
target_link_libraries(tray_detector ${LINK_LIBS})
# add_dependencies(grasp_tf_broadcaster ${grasp_tf_broadcaster_EXPORTED_TARGETS})
