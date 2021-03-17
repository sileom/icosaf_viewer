cmake_minimum_required(VERSION 2.8.3)
project(icosaf_viewer)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

#find_package(OpenCV)
find_package(OpenCV REQUIRED#4.2 REQUIRED
  COMPONENTS
  opencv_highgui
  opencv_objdetect
  CONFIG
)
#include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package()

include_directories(
  include
  src/
  src/imbs
  ${catkin_INCLUDE_DIRS}
)

set(SRCS src/icosaf_viewer.cpp
         src/imbs/imbs.cpp
         src/imbs/imagemanager.cpp)

add_executable(icosaf_viewer ${SRCS})

target_link_libraries(icosaf_viewer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

