cmake_minimum_required(VERSION 2.8.3)
project(formation_trajectory)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  uav_msgs
)
catkin_package(
)
include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/main.cpp src/cicle1.cpp src/cicle2.cpp src/collaborate.cpp)
add_dependencies(${PROJECT_NAME}_node uav_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
