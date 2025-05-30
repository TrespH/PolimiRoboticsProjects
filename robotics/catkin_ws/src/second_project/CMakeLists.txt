cmake_minimum_required(VERSION 3.0.2)
project(second_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  nav_msgs
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
   nav_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_to_tf src/odom_to_tf.cpp)
add_executable(laser_scan_merger src/laser_scan_merger.cpp)


add_dependencies(odom_to_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(laser_scan_merger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(odom_to_tf ${catkin_LIBRARIES})
target_link_libraries(laser_scan_merger ${catkin_LIBRARIES})