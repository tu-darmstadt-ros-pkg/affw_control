cmake_minimum_required(VERSION 2.8.3)
project(affw_ctrl)

add_subdirectory(lib)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	affw_msgs
	message_filters
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS roscpp affw_msgs
)

include_directories(${affw_INCLUDE})
add_executable(affw_ctrl src/affw_ctrl.cpp)
target_link_libraries(affw_ctrl ${catkin_LIBRARIES} affw)
add_dependencies(affw_ctrl affw affw_msgs_generate_messages_cpp)
