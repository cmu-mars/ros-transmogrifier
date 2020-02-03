set -e
set -x

ROS1_INSTALL_LOCATION=$ROS_ROOT/../..

OPTIMIZE=-O3

g++ $OPTIMIZE -Wall -shared -std=c++11 -fPIC `pkg-config --cflags python2` ros/ros1/cpp/node_init.cpp -o node_init`python-config --extension-suffix` -I$ROS1_INSTALL_LOCATION/include -L$ROS1_INSTALL_LOCATION/lib -lroscpp
g++ $OPTIMIZE -Wall -shared -std=c++11 -fPIC `pkg-config --cflags python2` ros/ros1/cpp/ros_bindings.cpp -o ros_bindings`python-config --extension-suffix` -I$ROS1_INSTALL_LOCATION/include -L$ROS1_INSTALL_LOCATION/lib -lroscpp -lrostime -ltf2_ros -lkdl_parser -lurdf
g++ $OPTIMIZE -Wall -shared -std=c++11 -fPIC `pkg-config --cflags python2` ros/ros1/cpp/messages/geometry_msgs/TwistCppImpl.cpp -o ros/ros1/cpp/messages/geometry_msgs/TwistCppImpl`python-config --extension-suffix` -I$ROS1_INSTALL_LOCATION/include -L$ROS1_INSTALL_LOCATION/lib -lroscpp
g++ $OPTIMIZE -Wall -shared -std=c++11 -fPIC `pkg-config --cflags python2` ros/ros1/cpp/messages/nav_msgs/OdometryCppImpl.cpp -o ros/ros1/cpp/messages/nav_msgs/OdometryCppImpl`python-config --extension-suffix` -I$ROS1_INSTALL_LOCATION/include -L$ROS1_INSTALL_LOCATION/lib -lroscpp
g++ $OPTIMIZE -Wall -shared -std=c++11 -fPIC `pkg-config --cflags python2` ros/ros1/cpp/messages/sensor_msgs/JointStateCppImpl.cpp -o ros/ros1/cpp/messages/sensor_msgs/JointStateCppImpl`python-config --extension-suffix` -I$ROS1_INSTALL_LOCATION/include -L$ROS1_INSTALL_LOCATION/lib -lroscpp
