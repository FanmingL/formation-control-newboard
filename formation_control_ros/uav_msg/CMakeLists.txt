cmake_minimum_required(VERSION 2.8.3)
project(uav_msgs)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  std_srvs
)

 add_message_files(
   FILES
   uav_position.msg
   uav_information.msg
   uav_control.msg
 )

 add_service_files(
   FILES
   serial_srv.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
