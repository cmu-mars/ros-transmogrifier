set -e
set -x

ROS1_INSTALL_LOCATION=$ROS_ROOT/../..

OPTIMIZE=-O3

g++ $OPTIMIZE -Wall -shared -std=c++11 -fPIC `pkg-config --cflags python2` cpp/pdw_turtlebot3_fake/src/pdw_turtlebot3_fake.cpp -o CppTurtleBot3Fake`python-config --extension-suffix` -Icpp/pdw_turtlebot3_fake/include -I$ROS1_INSTALL_LOCATION/include

# Statically link CppRobotStatePublisher.o.
# Dynamically linking it doesn't work for some reason
# if you have py::object arguments in a function.
g++ $OPTIMIZE -c -Wall -shared -std=c++11 -fPIC `pkg-config --cflags python2` cpp/pdw_robot_state_publisher/src/pdw_robot_state_publisher.cpp -o CppRobotStatePublisher.o -Icpp/pdw_robot_state_publisher/include -I$ROS1_INSTALL_LOCATION/include
g++ $OPTIMIZE -Wall -shared -std=c++11 -fPIC `pkg-config --cflags python2` cpp/pdw_robot_state_publisher/src/pdw_joint_state_listener.cpp -o CppJointStateListener`python-config --extension-suffix` -Icpp/pdw_robot_state_publisher/include -I$ROS1_INSTALL_LOCATION/include CppRobotStatePublisher.o -L$ROS1_INSTALL_LOCATION/lib -lorocos-kdl
