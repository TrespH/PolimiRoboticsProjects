cmake_minimum_required(VERSION 2.8.3)
project(sub_asynch)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(standard_pub src/pub.cpp)
target_link_libraries(standard_pub ${catkin_LIBRARIES})

add_executable(standard_sub src/standard_sub.cpp)
target_link_libraries(standard_sub ${catkin_LIBRARIES})


add_executable(asynch_sub src/asynch_sub.cpp)
target_link_libraries(asynch_sub ${catkin_LIBRARIES})




