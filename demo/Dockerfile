FROM ubuntu:18.04

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y gnupg2 \
    locales \
    curl \
    git \
    openjdk-11-jdk \
    vim

# ROS1 installation
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(bash -c '\''source /etc/os-release; echo $UBUNTU_CODENAME'\'') main" \
		  > /etc/apt/sources.list.d/ros-latest.list'

RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt update && apt install -y ros-melodic-desktop-full

# ROS2 installation
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu $(bash -c '\''source /etc/os-release; echo $UBUNTU_CODENAME'\'') main" \
          > /etc/apt/sources.list.d/ros2-latest.list'

RUN apt update && apt install -y ros-dashing-desktop

ENV HOME /root
WORKDIR $HOME/brass_pdw
ADD rosdiscover_output $HOME/brass_pdw/rosdiscover_output

RUN git clone https://github.com/cmu-mars/arch2wy.git

ENV TERM xterm
RUN bash -c 'bash <(curl -sL get.comby.dev)'

RUN git clone https://github.com/pybind/pybind11.git
WORKDIR $HOME/brass_pdw/pybind11/build
RUN cmake ..
RUN make -j8
RUN make install

ADD wyvern $HOME/wyvern
ENV PATH="$PATH:$HOME/wyvern/bin"

WORKDIR $HOME/brass_pdw/teleop-demo
RUN python $HOME/brass_pdw/arch2wy/arch2wy.py $HOME/brass_pdw/rosdiscover_output/teleop_arch.yml TurtleBot3Teleop $HOME/brass_pdw/teleop-demo

COPY src $HOME/brass_pdw/teleop-demo/cpp
COPY pdw_turtlebot3_burger.urdf.xacro $HOME/brass_pdw/teleop-demo/cpp/pdw_turtlebot3_fake/urdf/
RUN bash -c "source /opt/ros/melodic/setup.bash; xacro \
             $HOME/brass_pdw/teleop-demo/cpp/pdw_turtlebot3_fake/urdf/pdw_turtlebot3_burger.urdf.xacro \
             -o $HOME/brass_pdw/teleop-demo/cpp/pdw_turtlebot3_fake/urdf/pdw_turtlebot3_burger.urdf base_dir:=$(pwd)"

COPY pdw_turtlebot3_teleop $HOME/brass_pdw/teleop-demo

COPY pdw_turtlebot3_fake_ros1.rviz $HOME/brass_pdw/teleop-demo/scripts/run/visualization/rviz/
COPY pdw_turtlebot3_fake_ros2.rviz $HOME/brass_pdw/teleop-demo/scripts/run/visualization/rviz/

# Transform CPP sources to a form that can be integrated with Wyvern
RUN bash scripts/comby/transform.sh

# Compile the ROS1 and ROS2 libraries
# Note: The output produced as a result of the -x flag is written to standard error.
#       Because of this, all this output is printed in red by docker which can be alarming
#       to users even though there might not be any issues. To avoid this, we are
#       redirecting standard error to standard output. The side effect of this is that
#       we have to scan through the printed text to find relevant information when an
#       error does occur.
RUN bash -c 'source /opt/ros/melodic/setup.bash; bash scripts/ros/compile_ros1_libs.sh 2>&1'
RUN bash -c 'source /opt/ros/dashing/setup.bash; bash scripts/ros/compile_ros2_libs.sh 2>&1'

# Compile CPP sources for ROS1 and ROS2
RUN bash -c 'source /opt/ros/melodic/setup.bash; bash scripts/compile/cpp/compile_for_ros1.sh 2>&1'
RUN bash -c 'source /opt/ros/dashing/setup.bash; bash scripts/compile/cpp/compile_for_ros2.sh 2>&1'

# Compile Wyvern sources
RUN bash -c 'bash scripts/compile/wyvern/compile_for_ros1.sh 2>&1'

ENTRYPOINT ["/bin/bash"]
