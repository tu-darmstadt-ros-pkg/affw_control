cmake_minimum_required(VERSION 2.8.3)
project(affw_ctrl)

include(ExternalProject)

add_subdirectory(lib/affw)
include_directories(lib/affw/include)

include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/lwpr/include/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/fann/include/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/otl/include/)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/lwpr/lib/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/fann/lib/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib/affw/otl/lib/)

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

include_directories(BEFORE ${affw_INCLUDES})
add_executable(affw_ctrl src/affw_ctrl.cpp)
target_link_libraries(affw_ctrl ${catkin_LIBRARIES} affw ${affw_LIBRARIES})
add_dependencies(affw_ctrl affw affw_msgs_generate_messages_cpp LIB_AFFW)
