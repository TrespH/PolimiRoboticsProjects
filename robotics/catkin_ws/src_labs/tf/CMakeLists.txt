cmake_minimum_required(VERSION 2.8.3)
project(tf_examples)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tf_pub src/pub.cpp)
target_link_libraries(tf_pub ${catkin_LIBRARIES})

add_executable(get_tf src/get_tf.cpp)
target_link_libraries(get_tf ${catkin_LIBRARIES})

