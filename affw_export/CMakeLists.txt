cmake_minimum_required(VERSION 2.8.3)
project(affw_export)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	affw_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  CATKIN_DEPENDS roscpp affw_msgs
)

add_executable(affw_export src/affw_export.cpp)
target_link_libraries(affw_export ${catkin_LIBRARIES})
add_dependencies(affw_export affw_msgs_generate_messages_cpp)
