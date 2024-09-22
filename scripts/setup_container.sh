#!/bin/bash
sudo apt-get install -y ros-foxy-xacro
sudo apt-get install -y ros-foxy-rviz2 
sudo apt-get install -y libgazebo11-dev gazebo11 ros-foxy-gazebo-ros ros-foxy-gazebo-ros-pkgs
sudo apt-get install -y ros-foxy-mouse-teleop
sudo apt upgrade -y

echo "source /ros_entrypoint.sh" >> ~/.bashrc
echo "SIM_SETUP=~/bike-sim/install/setup.bash" >> ~/.bashrc
echo "test -f \$SIM_SETUP && source \$SIM_SETUP" >> ~/.bashrc
source ~/.bashrc


sudo apt-get install -y \
    curl \
    git \
    zip \
    qtcreator \
    cmake \
    build-essential \
    genromfs \
    ninja-build \
    libopencv-dev \
    wget \
    python-argparse \
    python3-empy \
    python3-toml \
    python-numpy \
    python-dev \
    python3 \
    python3-pip \
    python-yaml \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-doc \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libprotobuf-dev \
    libprotoc-dev \
    protobuf-compiler \
    libeigen3-dev \
    libxml2-utils \
    python3-rospkg \
    python3-jinja2 \
    python3-numpy


pip3 install -U future 

python3 -m pip install pandas jinja2 pyserial pyulog pyyaml numpy toml empy packaging jsonschema future 

pip install --upgrade numpy


#Install Mavlink
echo "Step 4: Install MAVLINK"
cd /usr/include
git clone https://github.com/mavlink/c_library_v2.git --recursive
rm -rf /usr/include/c_library_v2/.git
mv /usr/include/c_library_v2/* /usr/include
rmdir /usr/include/c_library_v2




#Install PX4
echo "Step 6: INSTALLING PX4 Autopilot"
mkdir -p /px4_sitl
cd /px4_sitl
git clone https://github.com/PX4/PX4-Autopilot.git 
cd /px4_sitl/PX4-Autopilot
git checkout a6274bc
git submodule update --init --recursive

apt-get remove modemmanager -y

echo "Step 7: PX4 Installing dependencies"
wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -P /bin && \
    chmod a+x /bin/QGroundControl.AppImage

#Install mavros
echo "Step 8: Installing mavros"
apt-get install -y \
    ros-foxy-rqt \
    ros-foxy-rqt-common-plugins \
    ros-foxy-mavros \
    ros-foxy-mavros-extras

# Clone sitl
echo "Step 9: Cloning PX4 sitl"
cd /px4_sitl
git clone --recursive https://github.com/PX4/sitl_gazebo.git
mkdir /px4_sitl/sitl_gazebo/build
cd /px4_sitl/sitl_gazebo/build/

# Build Sitl
echo "Step 10: Building PX4 sitl"
CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/usr/bin/gazebo
cmake .. && make -j2 && make install

#Set some environment variables to get PX4 to build
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
Col