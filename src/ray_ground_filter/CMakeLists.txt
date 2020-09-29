cmake_minimum_required(VERSION 2.8.3)
project(ray_ground_filter)
add_compile_options (-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        tf
	)

catkin_package(
	CATKIN_DEPENDS 
	roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        tf
	)

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${catkin_INCLUDE_DIRS}
        ${autoware_config_msgs_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	include
	)
link_directories(${PCL_LIBRARY_DIRS})

# Ray Ground Filter
add_library(ray_ground_filter_lib SHARED
        ray_ground_filter.cpp)

target_include_directories(ray_ground_filter_lib PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        /include
        )

target_link_libraries(ray_ground_filter_lib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Qt5Core_LIBRARIES}
        )

add_executable(ray_ground_filter
        ray_ground_filter_main.cpp
        )

target_include_directories(ray_ground_filter PRIVATE
        /include)

target_link_libraries(ray_ground_filter
        ray_ground_filter_lib)

add_dependencies(ray_ground_filter ${catkin_EXPORTED_TARGETS})

