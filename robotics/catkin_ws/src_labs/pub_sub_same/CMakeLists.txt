cmake_minimum_required(VERSION 2.8.3)
project(pub_sub_same)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(test_pub src/test_pub.cpp)

add_dependencies(test_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_pub ${catkin_LIBRARIES})

add_executable(test_sub src/test_sub.cpp)

add_dependencies(test_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(test_sub ${catkin_LIBRARIES})
