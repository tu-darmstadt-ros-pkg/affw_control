cmake_minimum_required(VERSION 2.8.3)
project(affw_ctrl)

include(ExternalProject)

ExternalProject_Add(
	LIB_AFFW
	PREFIX affw
	URL ${CMAKE_CURRENT_SOURCE_DIR}/lib/affw
	INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/affw/src/LIB_AFFW-build/lwpr/include/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/affw/src/LIB_AFFW-build/fann/include/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/affw/src/LIB_AFFW-build/otl/include/)

link_directories(${CMAKE_CURRENT_BINARY_DIR}/lib)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/affw/src/LIB_AFFW-build/lwpr/lib/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/affw/src/LIB_AFFW-build/fann/lib/)
link_directories(${CMAKE_CURRENT_BINARY_DIR}/affw/src/LIB_AFFW-build/otl/lib/)

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
target_link_libraries(affw_ctrl ${catkin_LIBRARIES} affw lwpr fann OTL)
add_dependencies(affw_ctrl affw affw_msgs_generate_messages_cpp LIB_AFFW)
