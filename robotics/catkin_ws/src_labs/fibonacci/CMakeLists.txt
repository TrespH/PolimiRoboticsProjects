cmake_minimum_required(VERSION 2.8.3)
project(actionlib_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
)


## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES Fibonacci.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs  # Or other packages containing msgs
)

#
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(fibonacci_server src/fibonacci_server.cpp)

target_link_libraries(
  fibonacci_server
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_server
  ${actionlib_tutorials_EXPORTED_TARGETS}
)



add_executable(fibonacci_client src/fibonacci_client.cpp)

target_link_libraries( 
  fibonacci_client
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_client
  ${actionlib_tutorials_EXPORTED_TARGETS}
)

add_executable(fibonacci_client2 src/fibonacci_client2.cpp)

target_link_libraries( 
  fibonacci_client2
  ${catkin_LIBRARIES}
)

add_dependencies(
  fibonacci_client2
  ${actionlib_tutorials_EXPORTED_TARGETS}
)
