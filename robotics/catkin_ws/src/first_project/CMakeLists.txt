cmake_minimum_required(VERSION 3.0.2)
project(first_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
  message_generation
  nav_msgs
)

add_message_files(
   FILES
   sector_times.msg
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
  message_runtime
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(gps_odometer src/gps_odometer.cpp)

add_executable(odometer src/odometer.cpp)

add_executable(sector_times src/sector_times.cpp)

add_dependencies(sector_times ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sector_times first_project_generate_messages_cpp)

add_dependencies(gps_odometer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(gps_odometer
   ${catkin_LIBRARIES}
)

target_link_libraries(odometer
   ${catkin_LIBRARIES}
)

target_link_libraries(sector_times
   ${catkin_LIBRARIES}
)