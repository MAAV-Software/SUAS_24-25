Launching World:

Three different terminals needed

Terminal 1:
1. cd into Micro-XRCE-DDS-Agent
2. MicroXRCEAgent udp4 -p 8888

Terminal 2:
1. cd into PX4-Autopilot
2. make px4_sitl gz_x500
2.1. PX4_GZ_WORLD=<world name here> make px4_sitl gz_x500

Terminal 3:
1. cd into Downloads
2. ./QGroundControl.AppImage

Terminal 4:
1. cd into SUAS_24-25/software_ws
2. source /opt/ros/humble/setup.bash
3. colcon build --packages-select px4_ros_com 
4. source install/setup.bash 
5. ros2 run px4_ros_com offboard_control

---------------------------------------------------

Run gazebo with sdf (world) file
1. cd int PX4-Autopilot
2. PX4_GZ_WORLD=<world name here> make px4_sitl gz_x500

Note: If drone does not load into world file, check that world definition matches world name on line 3

---------------------------------------------------

Creating new random sdf (world) file
1. cd into SUAS_24-25/software_ws/src/waypoint_generation
2. python3 way_point_generation.py
3. move sdf file to /PX4-Autopilot/Tools/simulation/gz/worlds

---------------------------------------------------

Check ros2 topics for vehicle_local_position
1. source /opt/ros/humble/setup.bash 
2. ros2 topic
list
 Now find important topics like
fmu/out/vehicle_local_posiition
fmu/out/vehicle_gps_posiition
fmu/out/vehicle_global_position
fmu/in/trajectory_setpoint


---------------------------------------------------

Running on nuc (likely siimilar to jetson)
1. # Intel NUC Setup

1. Setup NUC: 
   a. Keyboard, HDMI cable, etc.
2. log in manually to NUC, (username='maav', password='get pwnd'). 
5. SHH into NUC by typing: 
    a. ssh maav@<ip_address> 
    b. password same as above: 'get pwnd'


Cd /home/maav/SUAS-2023/software_ws
source ./devel/setup.bash
rosrun suas suas_hover

suas.test_hover.cpp


Source /opt/ros/noetic/setup.bash
Roscore


---------------------------------------------------

Command for all 4 terminal setups:

./very_kool_script.sh [worldname]