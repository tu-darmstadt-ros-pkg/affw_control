cmake_minimum_required(VERSION 2.8.3)
project(export_movement_data)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs tf
)

add_executable(export_movement_data src/export_movement_data.cpp)
target_link_libraries(export_movement_data ${catkin_LIBRARIES})
