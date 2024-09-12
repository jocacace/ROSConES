#Base image used to create the new image
FROM ubuntu:jammy

# User and group ids of in the host machine
# To check their value use these commands
# $ id -u  (for the UID)
# $ id -g  (for the GUI)
ARG UID=1000
ARG GID=1000

RUN locale
RUN apt-get update && apt-get install locales -y
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
 
RUN apt-get install software-properties-common -y
RUN add-apt-repository universe

RUN apt update && \
    apt install -y sudo && \
    addgroup --gid $GID user && \
    adduser --uid $UID --gid $GID --disabled-password --gecos "" user && \
    echo 'user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

RUN apt-get install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" |  tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update
RUN apt-get upgrade -y

RUN DEBIAN_FRONTEND=noninteractive apt-get install \
ros-humble-desktop ros-dev-tools  \
python3-colcon-common-extensions \
mlocate  \
python3-colcon-clean -y

RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg 
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null 

RUN apt-get update 
RUN DEBIAN_FRONTEND=noninteractive apt-get install \
ignition-fortress  -y \
ros-humble-ros-ign-bridge -y \
ros-humble-ros-gz -y \
ros-humble-controller-manager -y \
ros-humble-ros2-control -y  \
ros-humble-ros2-controllers -y \
ros-humble-ign-ros2-control -y \
ros-humble-ign-ros2-control-demos -y \
ros-humble-joint-state-publisher-gui -y


RUN DEBIAN_FRONTEND=noninteractive apt-get install ros-humble-realsense2-description -y

USER user
CMD ["/bin/bash"]









