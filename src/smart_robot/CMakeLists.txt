cmake_minimum_required(VERSION 3.0.2)
project(smart_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  gaze_ros
  geometry_msgs  
  message_generation
  message_runtime
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

add_action_files(
  DIRECTORY action
  FILES MoveToPlant.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)

catkin_install_python(
  scripts/manager_node.py
  scripts/navigator_node.py
  scripts/pump_driver_node.py
  scripts/moisture_sensor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
)