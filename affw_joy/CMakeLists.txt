cmake_minimum_required(VERSION 2.8.3)
project(affw_joy)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs
)

add_library(traj src/angle_math.cpp src/traj_2order.cpp src/traj_generator.cpp)

add_executable(joy2vel src/joy2vel.cpp)
target_link_libraries(joy2vel ${catkin_LIBRARIES} traj)
