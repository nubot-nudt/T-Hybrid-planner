cmake_minimum_required(VERSION 3.0.2)
project(terrain_analysis)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++03)
#add_definitions(-std=c++03)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        pcl_conversions
        pcl_ros
        )
find_package(PCL 1.8.0 REQUIRED)

catkin_package( 
#pcl_convers
#  INCLUDE_DIRS include
#  LIBRARIES terrain_analysis
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(find_location src/find_location.cpp)
target_link_libraries (find_location ${PCL_LIBRARIES} )


add_executable (cloud_process src/cloud_process.cpp)
target_link_libraries (cloud_process ${PCL_LIBRARIES})


add_executable (find_location_new src/find_location_new.cpp)
target_link_libraries (find_location_new ${PCL_LIBRARIES}  ${catkin_LIBRARIES})


add_executable (terrain_analysis src/terrain_analysis.cpp)
target_link_libraries (terrain_analysis ${PCL_LIBRARIES}  ${catkin_LIBRARIES})





