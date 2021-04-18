FROM ubuntu:20.04

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt update && \
    apt install -q -y --no-install-recommends tzdata

# install packages
RUN apt install -q -y --no-install-recommends \
    dirmngr gnupg2

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

RUN apt update && apt install --no-install-recommends -y ros-${ROS_DISTRO}-desktop=1.5.0-1*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-sound-play \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro ${ROS_DISTRO}

ENV reset true

RUN apt update && apt install -y git curl net-tools
RUN git clone https://github.com/rieg-ec/dotfiles.git ~/dotfiles && cd ~/dotfiles && \
    chmod +x install.sh && \
    echo "export minimal=false\nexport install_docker=false\nexport install_node=true\nexport USER=root" >> env.sh && \
    /bin/bash ./install.sh

# to preview markdown with neovim plugin in the browser
RUN apt install -y xdg-utils --fix-missing

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# install third party packages
ENV third_party_ws /root/third_party_ws
RUN mkdir -p ${third_party_ws}/src && \
    cd ${third_party_ws}/src && /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash; catkin_init_workspace" && \
    git clone https://github.com/yujinrobot/yujin_ocs.git && \
    cp -r yujin_ocs/yocs_cmd_vel_mux yocs_cmd_vel_mux && rm -rf yujin_ocs && \
    git clone https://github.com/yujinrobot/yocs_msgs.git && \
    git clone https://github.com/sniekum/ar_track_alvar_msgs.git && \
    git clone https://github.com/rieg-ec/very_simple_robot_simulator.git && \
    cd .. && rosdep install -y --from-paths src/ --ignore-src --rosdistro ${ROS_DISTRO} && \
    /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make" && \
    echo "source ${third_party_ws}/devel/setup.bash --extend" >> ~/.bashrc

# dependency for very_simple_robot_simulator
RUN apt install -y python3-pil python3-pil.imagetk

# create catkin_ws
ENV catkin_ws /root/catkin_ws
RUN mkdir -p ${catkin_ws}

COPY ./src $catkin_ws/src

RUN cd ${catkin_ws}/src && /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash; catkin_init_workspace" && \
    cd .. && /bin/bash -c ". /opt/ros/${ROS_DISTRO}/setup.bash; catkin_make" && \
    echo "source ${catkin_ws}/devel/setup.bash --extend" >> ~/.bashrc

CMD ["bash"]
