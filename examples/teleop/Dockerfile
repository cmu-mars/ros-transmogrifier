FROM ubuntu:16.04

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive
ENV TERM xterm

# Fixes the following error:
# debconf: delaying package configuration, since apt-utils is not installed
RUN apt-get update && apt-get install -y --no-install-recommends apt-utils \
    # sudo needs to be installed because rosdep uses it to install system dependencies.
    sudo

# Install ROS as well as TurtleBot3 simulation and keyboard teleop packages.
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(bash -c '\''source /etc/os-release; echo $UBUNTU_CODENAME'\'') main" \
    > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get install -y build-essential \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool

# Running rosdep install ... gave me the following error:
#   ERROR: the following packages/stacks could not have their rosdep keys resolved
#   to system dependencies:
#   catkin: No definition of [python-argparse] for OS version [xenial]
#   rosgraph: No definition of [python-rospkg] for OS version [xenial]
#   ...
# When I ran rosdep resolve python-rospkg, I saw the following:
#   ERROR: No definition of [python-rospkg] for OS version [xenial]
#   ...
#   ubuntu:
#    '*':
#    - python-rospkg
#    trusty_python3:
#    - python3-rospkg
#    utopic_python3:
#    - python3-rospkg
#    vivid_python3:
#    - python3-rospkg
#    wily_python3:
#    - python3-rospkg
#    xenial_python3:
#    - python3-rospkg
#    yakkety_python3:
#    - python3-rospkg
#    zesty_python3:
#    - python3-rospkg
# So, the rosdep YAML has been changed to use wildcards. But the rosdep installed with
# Ubuntu 16.04 does not know how to handle wildcards. Great job breaking older versions
# of rosdep, rosdep devs!
# To fix this, I added the code to handle wildcards in the installed lookup.py file from
# the lookup.py file in rosdep's GitHub repo. This file is being copied to the image here.
COPY lookup.py /usr/lib/python2.7/dist-packages/rosdep2/

RUN rosdep init && rosdep update && apt-get update

ENV HOME /root
#RUN mkdir -p $HOME/ros1_src/kinetic
WORKDIR $HOME/ros1_src/kinetic

RUN rosinstall_generator desktop_full --rosdistro kinetic --deps --wet-only --tar > kinetic-desktop-full-wet.rosinstall \
    && rosinstall_generator turtlebot3 --rosdistro kinetic --deps --tar > turtlebot3.rosinstall \
    && rosinstall_generator turtlebot3_simulations --rosdistro kinetic --deps --tar > turtlebot3-simulations.rosinstall \
    && rosinstall_generator teleop_twist_keyboard --rosdistro kinetic --deps --tar > teleop-twist-keyboard.rosinstall \
    && wstool init -j8 src kinetic-desktop-full-wet.rosinstall \
    && wstool merge -t src turtlebot3.rosinstall \
    && wstool merge -t src turtlebot3-simulations.rosinstall \
    && wstool merge -t src teleop-twist-keyboard.rosinstall \
    && wstool update -t src \
    && rosdep install --from-paths src --ignore-src --rosdistro kinetic -y \
    # Fixes the error described in the link below that I was getting as well.
    # https://answers.ros.org/question/294780/ubuntu18-and-ros-melodic-catkin-error-importerror-no-module-named-terminal_color/
    && apt-get install -y python-catkin-pkg-modules \
    && ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release

ENV TURTLEBOT3_MODEL burger
RUN echo "source $HOME/ros1_src/kinetic/install_isolated/setup.bash" >> .bashrc

# The environment variable QT_X11_NO_MITSHM was added to solve the issue described here:
# https://github.com/osrf/docker_images/issues/21
ENV QT_X11_NO_MITSHM 1
# The environment variable SVGA_VGPU was added to solve the issue described here:
# http://answers.gazebosim.org/question/13214/virtual-machine-not-launching-gazebo/
ENV SVGA_VGPU10 0
ENTRYPOINT source $HOME/ros1_src/kinetic/install_isolated/setup.bash \
    && roslaunch turtlebot3_gazebo turtlebot3_world.launch

ENV DEBIAN_FRONTEND teletype
