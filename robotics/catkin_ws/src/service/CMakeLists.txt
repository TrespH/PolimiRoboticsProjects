cmake_minimum_required(VERSION 2.8.3)
project(service)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

add_service_files(FILES AddTwoInts.srv)

generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(add_two_int src/add_two_int.cpp)
target_link_libraries(add_two_int ${catkin_LIBRARIES})
add_dependencies(add_two_int ${catkin_EXPORTED_TARGETS})

add_executable(client src/client.cpp)
target_link_libraries(client ${catkin_LIBRARIES})
add_dependencies(client ${catkin_EXPORTED_TARGETS})

