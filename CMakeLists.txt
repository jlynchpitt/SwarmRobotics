cmake_minimum_required(VERSION 2.8.3)
project(swarm)

find_package(catkin REQUIRED COMPONENTS 
    rospy
    std_msgs
    message_generation)

add_message_files(
	FILES
	SensorData.msg
	RobotVelocity.msg
	WheelSpeeds.msg
    RobotLocation.msg
    RobotLocationList.msg
)

#add_service_files()

generate_messages(
	DEPENDENCIES
	std_msgs
)


catkin_package(
    CATKIN_DEPENDS message_runtime
)