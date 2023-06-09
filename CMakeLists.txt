cmake_minimum_required(VERSION 2.8.3)
project(livox_laser_simulation)

## Compile as C++11, supported in ROS Kinetic and newer





set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

set(CMAKE_CXX_STANDARD_LIBRARIES "-lstdc++")
find_package(GTSAM REQUIRED)
find_package(GTSAMCMakeTools REQUIRED)

find_package(PCL REQUIRED QUIET)
find_package(Boost REQUIRED QUIET)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})

#To solve the error which gazebo does not handle the include file well, we must add this line.
include_directories(/usr/include/gazebo-7/gazebo)

link_directories(${GAZEBO_LIBRARY_DIRS})

#This is designed for whose proto installed in system is not 2.6. We can install the version of proto in local dir
#include_directories(/home/lfc/proto/include/)
#link_directories(/home/lfc/proto/lib/)

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  cv_bridge
  image_transport
  random_numbers
  tf
  pcl_conversions
  pcl_ros
  gazebo_msgs
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES livox_laser_simulation
        CATKIN_DEPENDS tf
	DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
    ${EIGEN_3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
    ${GTSAMCMakeTools_INCLUDE_DIRECTORIES}
)

add_library(livox_laser_simulation SHARED src/livox_points_plugin.cpp src/livox_ode_multiray_shape.cpp)
target_link_libraries(livox_laser_simulation ${catkin_LIBRARIES} RayPlugin)


target_link_libraries(livox_laser_simulation libprotobuf.so.9)

add_library(cliffordPlugin src/cliffordPlugin.cpp)
target_link_libraries(cliffordPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})



install(TARGETS cliffordPlugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(Imu_clifford_odom src/Imu_clifford_odom.cpp)
target_link_libraries(Imu_clifford_odom ${catkin_LIBRARIES} gtsam gtsam_unstable)








