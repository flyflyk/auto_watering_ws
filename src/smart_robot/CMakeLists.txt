cmake_minimum_required(VERSION 3.0.2)
project(smart_robot)

## 尋找所有必要的 Catkin 依賴項
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

## 增加 Action 檔案
add_action_files(
  DIRECTORY action
  FILES MoveToPlant.action
)

## 生成訊息 (包含 Actions)
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)

## Catkin Package 配置
catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES smart_robot
  CATKIN_DEPENDS 
    actionlib 
    actionlib_msgs 
    message_runtime 
    rospy 
    std_msgs
  #DEPENDS system_lib
)

## 指定要安裝的 Python 腳本
catkin_install_python(PROGRAMS
  scripts/manager_node.py
  scripts/navigator_node.py
  scripts/pump_driver_node.py
  scripts/moisture_sensor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)ls

## 指定要安裝的 Launch 和其他檔案
install(DIRECTORY launch urdf worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)