FROM ros:foxy

ARG USERNAME=username
ARG PASSWORD=password

ENV DEBIAN_FRONTEND noninteractive

SHELL ["/bin/bash", "-c"]

# install ros package
RUN sudo apt-get update -y && \
    apt-get install -y \
        openssh-server \
        lightdm \
        ubuntu-desktop \
        nano \
        xterm \
        x11vnc \ 
        xvfb \
        xorg \
        xfce4

RUN touch ~/.Xauthority

RUN echo "sudo service ssh start" >> ~/.bashrc && \
    echo "export TERM=xterm" >> ~/.bashrc && \
    echo "export ROS_DISTRO=foxy" >> ~/.bashrc && \
    echo "xauth add \$(runuser -l $USERNAME -c \"xauth list \$DISPLAY\") || echo 'xauth not set; restart terminal to run simulation'" >> ~/.bashrc

RUN sudo useradd -m -p $(openssl passwd -1 "$PASSWORD") $USERNAME && \
    usermod -aG sudo $USERNAME && \
    usermod -s /bin/bash $USERNAME