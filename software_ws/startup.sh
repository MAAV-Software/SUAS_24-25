#!/bin/bash

gnome-terminal --tab --title="Micro-XRCE-DDS-Agent" -- bash -c "cd ~/Micro-XRCE-DDS-Agent && ./MicroXRCEAgent udp4 -p 8888; exec bash" \
               --tab --title="PX4 SITL" -- bash -c "cd ~/PX4-Autopilot && make px4_sitl gz_x500; PX4_GZ_WORLD=airportWaypoints make px4_sitl gz_standard_vtol; exec bash" \
               --tab --title="QGroundControl" -- bash -c "cd ~/Downloads && ./QGroundControl.AppImage; exec bash" \
               --tab --title="ROS2 PX4 Interface" -- bash -c "cd ~/SUAS_24-25/software_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select px4_ros_com && source install/setup.bash && ros2 run px4_ros_com offboard_control; exec bash"
