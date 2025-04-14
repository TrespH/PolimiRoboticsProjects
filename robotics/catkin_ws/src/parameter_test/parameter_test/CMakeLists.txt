cmake_minimum_required(VERSION 2.8.3)
project(parameter_test)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs dynamic_reconfigure)

generate_dynamic_reconfigure_options(cfg/parameters.cfg)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(param_first src/param_first.cpp)
target_link_libraries(param_first ${catkin_LIBRARIES})

add_executable(param_second src/param_second.cpp)
add_dependencies(param_second ${PROJECT_NAME}_gencfg)
target_link_libraries(param_second ${catkin_LIBRARIES})