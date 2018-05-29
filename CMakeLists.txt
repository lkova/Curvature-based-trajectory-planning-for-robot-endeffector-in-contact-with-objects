cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(schunk_lwa4p_force)

find_package(Boost REQUIRED
          system
          filesystem
          date_time
          thread)

find_package(catkin REQUIRED COMPONENTS
          pcl_ros
          roscpp
          rospy
          roslib
          cmake_modules
          geometry_msgs
          sensor_msgs
          std_msgs
	  schunk_lwa4p_trajectory
	  gazebo_ros
)



catkin_package(
    INCLUDE_DIRS src include config
    LIBRARIES  ${Boost_LIBRARIES} yaml-cpp
    ## LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS  geometry_msgs roscpp rospy std_msgs sensor_msgs
    DEPENDS system_lib boost roscpp gazebo_ros
)

#dodano
find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_library(point_cloud_position src/point_cloud_position.cpp)
target_link_libraries(point_cloud_position ${catkin_LIBRARIES})

#dodano
add_library(force_applied_plugin src/force_applied.cpp)
target_link_libraries(force_applied_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(pointCloudPositionNode nodes/pointCloudPositionNode.cpp)
target_link_libraries(pointCloudPositionNode point_cloud_position ${PCL_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(pointCloudPositionNode ${catkin_EXPORTED_TARGETS})

