cmake_minimum_required(VERSION 2.8.3)
project(sort_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_filters
  people_msgs
  roscpp
  rospy
  std_msgs
  tf
)
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES sort_tracking
 CATKIN_DEPENDS cv_bridge image_geometry image_transport message_filters people_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)


include_directories(
include
  SYSTEM
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(sort_tracking src/people_tracking_sort.cpp src/Sort.cpp src/Hungarian.cpp src/KalmanTracker.cpp src/KalmanTracker3d.cpp)
target_link_libraries(sort_tracking ${OpenCV_LIBS} ${catkin_LIBRARIES})
