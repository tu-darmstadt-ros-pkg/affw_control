cmake_minimum_required(VERSION 2.8.3)
project(affw_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
	FILES
	State.msg
	TargetRequest.msg
	ProcTime.msg
)

add_service_files(
   FILES
   ActionRequest.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

install(DIRECTORY include/affw_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")