cmake_minimum_required(VERSION 2.8.3)
project(Swarm)

find_package(catkin REQUIRED COMPONENTS message_generation rospy ...)

add_message_files()

add_service_files()

generate_messages()


catkin_package(
    CATKIN_DEPENDS message_runtime
)