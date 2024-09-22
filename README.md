1. `docker build . -t bike-sim` (bike-safe-and-save folder)
2. `docker run -it -p 22:22 --mount type=bind,source=$(pwd),target=/root/bike-sim --name BIKE bike-sim` (bike-safe-and-save folder)
3. `ssh -L 5900:localhost:5900 username@$(ipconfig getifaddr en0) "x11vnc -create -nopw -listen 127.0.0.1 -localhost"` (any terminal)
4. (VNC into localhost:5900)
5. `startxfce4` (inside container)
6. `sudo -i` (inside container; password is password)
6. `bash setup_container.sh` (inside container from bike-sim folder)
7. (Restart container terminal)
8. cd /px4_sitl/PX4-Autopilot/
9. make px4_sitl gazebo