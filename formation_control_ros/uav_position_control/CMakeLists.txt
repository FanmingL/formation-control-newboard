cmake_minimum_required(VERSION 2.8.3)
project(uav_position_control)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
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
add_executable(${PROJECT_NAME}_node src/main.cpp)
add_dependencies(${PROJECT_NAME}_node uav_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
