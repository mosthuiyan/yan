cmake_minimum_required(VERSION 3.5)
project(yan-slam)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  image_transport
  cv_bridge
  open3d_conversions
)

find_package(Open3D REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp rospy sensor_msgs
  DEPENDS system_lib
)

include_directories(include) 
include_directories(${EIGEN3_INCLUDE_DIR}) 
include_directories(${Open3D_LIBRARY_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

add_executable(kitti360_helper src/kitti360_helper.cc)
target_link_libraries(kitti360_helper ${catkin_LIBRARIES} ${Open3D_LIBRARIES} ${OpenCV_LIBS})