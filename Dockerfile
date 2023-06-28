FROM ros:galactic

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/usr/bin/bash", "-c"]

RUN apt-get update && \
  apt-get install -y --no-install-recommends \
  vim \
  tmux \
  wget \ 
  tree \
  clang-format

# rviz
RUN apt-get install -y \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-rqt-common-plugins


# rviz plugin dependencies
RUN apt-get install -y \
  qt5-default \ 
  libboost-all-dev

## install vicon receiver
WORKDIR /root/colcon_ws/src
RUN git clone https://github.com/dasc-lab/ros2-vicon-receiver.git
WORKDIR /root/colcon_ws/src/ros2-vicon-receiver
RUN ./install_libs.sh


# ## install octomap
# RUN apt-get install -y \
#   ros-${ROS_DISTRO}-octomap \ 
#   # ros-${ROS_DISTRO}-octomap-ros \
#   ros-${ROS_DISTRO}-octomap-mapping 
#   # ros-${ROS_DISTRO}-octomap-rviz-plugins \
#   # ros-${ROS_DISTRO}-octomap-msgs

## install python tf utils
RUN apt-get update && apt-get install python3-pip -y --no-install-recommends
RUN pip3 install transforms3d

# install pcl
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-pcl-ros 


# default locations
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/colcon_ws
