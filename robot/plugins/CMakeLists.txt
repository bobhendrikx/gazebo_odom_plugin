cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs)
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${roscpp_INCLUDE_DIRS})


link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(odom SHARED odom.cpp)
target_link_libraries(odom ${GAZEBO_LIBRARIES} )
target_link_libraries(odom ${catkin_LIBRARIES} )