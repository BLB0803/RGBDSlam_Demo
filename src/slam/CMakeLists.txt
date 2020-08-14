cmake_minimum_required(VERSION 2.8.3)
project(slam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  message_filters
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs image_transport sensor_msgs cv_bridge message_filters geometry_msgs
)


find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)

set(LIBS_SLAM
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

set(OPENCV_ENABLE_NONFREE = ON)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(
  ${PCL_DEFINITIONS}
  ${EIGEN_DEFINITIONS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/base.cpp src/frame.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${LIBS_SLAM}
)



