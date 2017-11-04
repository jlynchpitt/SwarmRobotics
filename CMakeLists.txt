cmake_minimum_required(VERSION 2.8.3)
project(Swarm)

find_package(catkin REQUIRED COMPONENTS message_generation rospy ...)

add_message_files(
	FILES
	SensorData.msg
)

add_service_files()

generate_messages(
	DEPENDENCIES
	std_msgs
)


catkin_package(
    CATKIN_DEPENDS message_runtime
)