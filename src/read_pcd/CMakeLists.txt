cmake_minimum_required(VERSION 3.0.2)
project(read_pcd)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  nav_msgs
  tf
)
find_package(PCL 1.8 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES read_pcd
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(read_pcd src/read_pcd.cpp)
target_link_libraries(read_pcd ${catkin_LIBRARIES})
add_executable(pcd_to_grid src/pcd_to_grid.cpp)
target_link_libraries(pcd_to_grid ${catkin_LIBRARIES} ${PCL_LIBRARIES})
