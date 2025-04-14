cmake_minimum_required(VERSION 2.8.3)
project(timer)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(timed_pub src/pub.cpp)
target_link_libraries(timed_pub ${catkin_LIBRARIES})



