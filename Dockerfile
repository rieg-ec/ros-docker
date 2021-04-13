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

RUN apt update && apt install --no-install-recommends -y ros-$ROS_DISTRO-desktop=1.5.0-1*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-$ROS_DISTRO-sound-play \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

ENV reset false

RUN apt update && apt install -y git curl net-tools
RUN git clone https://github.com/rieg-ec/dotfiles.git ~/dotfiles && cd ~/dotfiles && \
    chmod +x install.sh && \
    echo "export minimal=false\nexport install_docker=false\nexport install_node=true\nexport USER=root" >> env.sh && \
    /bin/bash ./install.sh

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

ENV pkg_dir /opt/ros/$ROS_DISTRO/share

# build yocs_cmd_vel_mux package
RUN mkdir -p $pkg_dir/yocs_cmd_vel_mux/src && cd $pkg_dir/yocs_cmd_vel_mux/src && \ 
    git clone https://github.com/yujinrobot/yujin_ocs.git && \
    cp -r yujin_ocs/yocs_cmd_vel_mux yocs_cmd_vel_mux && rm -rf yujin_ocs && \
    cd .. && rosdep install -y --from-paths src/ --ignore-src --rosdistro $ROS_DISTRO && \
    /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash; catkin_make" && \ 
    echo "source $pkg_dir/yocs_cmd_vel_mux/devel/setup.bash" >> ~/.bashrc


# create catkin_ws
ENV catkin_ws ~/catkin_ws
RUN mkdir -p catkin_ws/src && cd catkin_ws/src && /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash; catkin_init_workspace"


COPY ./src $catkin_ws/src 

RUN cd catkin_ws && rosdep install -y --from-paths src/ --ignore-src --skip-keys=yocs_cmd_vel_mux --rosdistro $ROS_DISTRO && \
    /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash; catkin_make" 

# build robot simulator package
# RUN mkdir catkin_ws/src && cd $catkin_ws/src && git clone https://github.com/gasevi/very_simple_robot_simulator.git && \
#     cd .. && rosdep install -y --from-paths src/ --ignore-src --skip-keys=yocs_cmd_vel_mux --rosdistro $ROS_DISTRO && \
#     /bin/bash -c ". /opt/ros/$ROS_DISTRO/setup.bash; catkin_make" 

RUN echo "source $catkin_ws/devel/setup.bash" >> ~/.bashrc

# dependency for very_simple_robot_simulator
# RUN apt install -y python2 && ln -s /usr/bin/python2 /usr/bin/python

# install pip2
# RUN sh -c ' curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py' && python2 get-pip.py

# any bash command must be written in here for persistance
COPY ./volume/setup.bash /setup.bash
RUN chmod +x /setup.bash

RUN echo "source /setup.bash" >> ~/.bashrc

CMD ["bash"]
