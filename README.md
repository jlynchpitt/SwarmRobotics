# SwarmRobotics

## Steps to install ros code
1. create directory /catkin_ws/src/swarm
2. create /swarm as git repo - download code
3. go to /catkin_ws
4. run catkin_make or catkin_make --force
5. run source devel/setup.bash
6. go to /catkin_ws/src/swarm
7. type python
8. type from swarm.msg import SensorData
9. confirm no messages
10. If failed try rosmsg list
  search for swarm/SensorData + any other custom messages
