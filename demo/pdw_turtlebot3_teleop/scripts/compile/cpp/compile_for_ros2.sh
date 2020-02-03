set -e
set -x

ROS2_INSTALL_LOCATION=$AMENT_PREFIX_PATH

OPTIMIZE=-O3

g++ $OPTIMIZE -Wall -shared -std=c++14 -fPIC `pkg-config --cflags python3` cpp/pdw_turtlebot3_fake/src/pdw_turtlebot3_fake.cpp -o CppTurtleBot3Fake`python3-config --extension-suffix` -Icpp/pdw_turtlebot3_fake/include -I$ROS2_INSTALL_LOCATION/include

g++ $OPTIMIZE -c -Wall -shared -std=c++14 -fPIC `pkg-config --cflags python3` cpp/pdw_robot_state_publisher/src/pdw_robot_state_publisher.cpp -o CppRobotStatePublisher.o -Icpp/pdw_robot_state_publisher/include -I$ROS2_INSTALL_LOCATION/include
g++ $OPTIMIZE -Wall -shared -std=c++14 -fPIC `pkg-config --cflags python3` cpp/pdw_robot_state_publisher/src/pdw_joint_state_listener.cpp -o CppJointStateListener`python3-config --extension-suffix` -Icpp/pdw_robot_state_publisher/include -I$ROS2_INSTALL_LOCATION/include CppRobotStatePublisher.o -L$ROS2_INSTALL_LOCATION/lib -lorocos-kdl
