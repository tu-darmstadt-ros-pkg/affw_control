cmake_minimum_required(VERSION 2.8.3)
project(affw_ctrl)

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

include_directories(src/learner)
add_library(model_learner src/learner/LWPRLearner.cpp src/learner/DummyLearner.cpp src/learner/ModelLearner.cpp src/learner/Config.cpp)
target_link_libraries(model_learner lwpr)

add_executable(affw_ctrl src/affw_ctrl.cpp)
target_link_libraries(affw_ctrl ${catkin_LIBRARIES} model_learner)
add_dependencies(affw_ctrl affw_msgs_generate_messages_cpp)
