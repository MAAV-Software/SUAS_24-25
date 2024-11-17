Launching World:

Three different terminals needed

Terminal 1:
1. cd into Micro-XRCE-DDS-Agent
2. MicroXRCEAgent udp4 -p 8888

Terminal 2:
1. cd into PX4-Autopilot
2. make px4_sitl gz_x500

Terminal 3:
1. cd into Downloads
2. ./QGroundControl.AppImage

Terminal 4:
1. cd into SUAS_24-25/software_ws
2. colcon build --packages-select px4_ros_com
3. source /opt/ros/humble/setup.bash 
4. source install/setup.bash 
5. ros2 run px4_ros_com offboard_control

---------------------------------------------------

Run gazebo with sdf (world) file
1. cd int PX4-Autopilot
2. PX4_GZ_WORLD=<world name here> make px4_sitl gz_x500

Note: If drone does not load into world file, check that world definition matches world name on line 3

---------------------------------------------------
