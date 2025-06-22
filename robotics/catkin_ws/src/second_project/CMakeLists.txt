cmake_minimum_required(VERSION 3.0.2)
project(second_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  move_base_msgs
  move_base
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  actionlib
  move_base_msgs
  move_base
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odom_to_tf mapping/odom_to_tf.cpp)
add_executable(laser_scan_merger mapping/laser_scan_merger.cpp)
add_executable(goal_publisher navigation/goal_publisher.cpp) 
add_executable(crop_map map/crop_map.cpp)

add_dependencies(odom_to_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(laser_scan_merger ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(goal_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(crop_map ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(odom_to_tf ${catkin_LIBRARIES})
target_link_libraries(laser_scan_merger ${catkin_LIBRARIES})
target_link_libraries(goal_publisher ${catkin_LIBRARIES})

target_link_libraries(crop_map
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)