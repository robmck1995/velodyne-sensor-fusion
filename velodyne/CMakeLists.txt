cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(velodyne_grabber)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (velodyne_grabber velodyne_grabber.cpp)
target_link_libraries (velodyne_grabber ${PCL_LIBRARIES})

add_executable (velodyne_static_capture simple_grabber.cpp simple_grabber.h velodyne_static_capture.cpp)
target_link_libraries (velodyne_static_capture ${PCL_LIBRARIES})
