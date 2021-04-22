cmake_minimum_required(VERSION 3.0.2)
project(pcl_tools)

set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
        message_filters
        pcl_conversions
        pcl_ros
        roscpp
        rospy
        sensor_msgs
        std_msgs
        )

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES pcl_tools
        #  CATKIN_DEPENDS message_filters pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs
        #  DEPENDS system_lib
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(box_filter src/box_filter.cpp)
target_link_libraries(box_filter ${catkin_LIBRARIES})

add_executable(transform src/transform.cpp)
target_link_libraries(transform ${catkin_LIBRARIES})

add_executable(cloud_fusion src/cloud_fusion.cpp)
target_link_libraries(cloud_fusion ${catkin_LIBRARIES})