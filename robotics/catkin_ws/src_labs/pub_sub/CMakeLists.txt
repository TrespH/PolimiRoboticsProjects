cmake_minimum_required(VERSION 2.8.3)

## Folder name not linked!
project(pub_sub)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

## 'pub' as the name of the node!
add_executable(pub src/pub.cpp)
target_link_libraries(pub ${catkin_LIBRARIES})

add_executable(sub src/sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})

