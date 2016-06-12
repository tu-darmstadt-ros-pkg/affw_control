cmake_minimum_required(VERSION 2.8.3)
project(affw_pos2vel)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  CATKIN_DEPENDS roscpp nav_msgs tf
)

add_executable(affw_pos2vel src/affw_pos2vel_msf.cpp)
target_link_libraries(affw_pos2vel ${catkin_LIBRARIES})
