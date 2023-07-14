FROM ros:galactic

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/usr/bin/bash", "-c"]

RUN apt-get update && \
  apt-get install -y --no-install-recommends \
  vim \
  tmux \
  wget \ 
  tree \
  clang-format \
  curl \ 
  clang

# install vimplug
RUN curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim

# rviz and gui dependencies
RUN apt-get install -y \
  ros-${ROS_DISTRO}-rviz2 \ 
  qt5-default \ 
  libboost-all-dev \
  libcanberra-gtk-module

# image transport
RUN apt-get install -y \ 
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-theora-image-transport \ 
  ros-${ROS_DISTRO}-rqt-common-plugins


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

# install realsense
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-realsense2-*

# install fastdds
RUN apt-get install -y ros-${ROS_DISTRO}-rmw-fastrtps-cpp

# install osqp
RUN pip3 install --upgrade numpy scipy osqp


# install ros joy
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-joy


# install julia
WORKDIR /root
RUN wget https://julialang-s3.julialang.org/bin/linux/x64/1.9/julia-1.9.2-linux-x86_64.tar.gz
RUN tar zxvf julia-1.9.2-linux-x86_64.tar.gz
RUN ln -s /root/julia-1.9.2/bin/julia /usr/local/bin/julia

## create a julia sysimage
COPY create_julia_sysimage.jl /root/create_julia_sysimage.jl
RUN julia /root/create_julia_sysimage.jl
RUN echo "alias julia='julia -J/root/JuliaSysImage.so'" >> /root/.bashrc

# julia precompile some functions
COPY colcon_ws/src/gatekeeper/gatekeeper/Project.toml /root/colcon_ws/src/gatekeeper/gatekeeper/Project.toml
COPY colcon_ws/src/gatekeeper/gatekeeper/Manifest.toml /root/colcon_ws/src/gatekeeper/gatekeeper/Manifest.toml
WORKDIR /root/colcon_ws/src/gatekeeper/gatekeeper
RUN julia --sysimage /root/JuliaSysImage.so --project -e 'using Pkg; Pkg.instantiate(); Pkg.precompile()'



# install the vimrc
COPY vimrc /root/.vimrc
RUN vim -E -s -u "$HOME/.vimrc" +PlugInstall +qall


# default locations
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/colcon_ws
