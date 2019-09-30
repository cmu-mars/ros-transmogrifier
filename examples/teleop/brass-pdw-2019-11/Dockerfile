FROM ubuntu:18.04

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y gnupg2

# Install ROS.
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(bash -c '\''source /etc/os-release; echo $UBUNTU_CODENAME'\'') main" \
    > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get install -y ros-melodic-desktop-full

# Install build tools.
RUN apt-get install -y build-essential \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool

RUN rosdep init && rosdep update && apt-get update

ENV HOME /root
RUN mkdir -p $HOME/brass_pdw/src
WORKDIR $HOME/brass_pdw

# Copy packages.
ADD pdw_robot_state_publisher src/pdw_robot_state_publisher
ADD pdw_turtlebot3_fake src/pdw_turtlebot3_fake
ADD pdw_turtlebot3_teleop src/pdw_turtlebot3_teleop

# Build and install the packages.
RUN bash -c 'source /opt/ros/melodic/setup.bash && catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release'

ENV TURTLEBOT3_MODEL burger

# add entrypoint
RUN echo "#!/bin/bash \n\
set -e \n\
source \"/opt/ros/melodic/setup.bash\" \n\
source \"/root/brass_pdw/install_isolated/setup.bash\" \n\
exec \"\$@\"" > "/entrypoint.sh" \
 && chmod +x "/entrypoint.sh"
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
