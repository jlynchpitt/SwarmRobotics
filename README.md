# SwarmRobotics

## Steps to install ros code
1. create directory ~/catkin_ws/src/swarm  
..* `mkdir -p ~/catkin_ws/src/; cd $_`
2. create /swarm as git repo - download code
..* `git clone https://github.com/jlynchpitt/SwarmRobotics; mv SwarmRobotics/ swarm/`
3. go to ~/catkin_ws
..* `cd ../`
4. run catkin_make or catkin_make --force
..* `catkin_make --force`
5. run source devel/setup.bash and add it to your .bashrc
..* `source devel/setup.bash; echo "source devel/setup.bash" >> ~/.bashrc`
6. go to ~/catkin_ws/src/swarm
..* `cd src/swarm`
7. type python
..* `python`
8. type `from swarm.msg import SensorData`
9. confirm no messages
10. If failed try rosmsg list
  search for swarm/SensorData + any other custom messages

