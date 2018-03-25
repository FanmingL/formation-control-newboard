cmake_minimum_required(VERSION 2.8.3)
project(uav_localization)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  uav_msgs
)
find_package(OpenCV REQUIRED)
catkin_package(
)
include_directories(
  include/
  /usr/local/include
  /usr/local/include/ocam
  /usr/local/include/apriltag
  ${catkin_INCLUDE_DIRS}
)
add_executable(${PROJECT_NAME}_node src/main.cpp)
add_dependencies(${PROJECT_NAME}_node uav_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node
  /usr/local/lib/libapriltag.so
  /usr/local/lib/libcam.so
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
