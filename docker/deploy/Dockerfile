FROM ros:galactic-ros-base

# create workspace
RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws/

# clone projects
RUN cd src && git clone https://github.com/IRobot-Algorithm/Vision_Control.git --depth=1 

# install dependencies and some tools
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y && \
    apt-get install wget htop vim can-utils libudev-dev libeigen3-dev usbutils -y && \
    sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen && \
    rm -rf /var/lib/apt/lists/*

# change terminal font color
RUN echo 'export PS1="\[\e[36m\]\u\[\e[m\]@\[\e[32m\]\h\[\e[m\]:\[\e[33m\]\w\[\e[m\]\$ "' >> /etc/bash.bashrc && \
    echo "export GREP_OPTIONS='--color=auto'" >> /etc/bash.bashrc && \
    echo "export GREP_COLOR='1;35'" >> /etc/bash.bashrc && \
    echo "alias ls='ls --color=auto'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_mb=$'\E[01;31m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_md=$'\E[01;31m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_me=$'\E[0m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_se=$'\E[0m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_so=$'\E[01;44;33m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_ue=$'\E[0m'" >> /etc/bash.bashrc && \
    echo "export LESS_TERMCAP_us=$'\E[01;32m'" >> /etc/bash.bashrc \
    echo "source /etc/bash.bashrc" >> ~/.bashrc

# setup libusb
WORKDIR /tmp 
RUN wget https://github.com/libusb/libusb/releases/download/v1.0.26/libusb-1.0.26.tar.bz2 
RUN sudo tar -jxvf libusb-1.0.26.tar.bz2 
WORKDIR /tmp/libusb-1.0.26
RUN sudo ./configure && \
    sudo make install

# build
WORKDIR /ros_ws/
RUN . /opt/ros/galactic/setup.sh && colcon build 

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "/ros_ws/install/setup.bash"' \
      /ros_entrypoint.sh
