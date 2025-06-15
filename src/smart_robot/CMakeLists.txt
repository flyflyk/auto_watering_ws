cmake_minimum_required(VERSION 3.0.2)
project(smart_robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  amcl
  gazebo_ros
  geometry_msgs
  gmapping
  map_server
  move_base
  move_base_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  xacro
)

add_service_files(
  FILES
  GetMoisture.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_install_python(PROGRAMS
  scripts/manager_node.py
  scripts/moisture_sensor_node.py
  scripts/pump_driver_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  urdf/
  config/
  worlds/
  maps/
  srv/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

