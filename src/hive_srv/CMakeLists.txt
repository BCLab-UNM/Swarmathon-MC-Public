cmake_minimum_required(VERSION 2.8.3)
project(hive_srv)

find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  message_generation
)

add_service_files(
  FILES
  hiveSrv.srv
  hiveAddRobot.srv
  calibrate.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)


