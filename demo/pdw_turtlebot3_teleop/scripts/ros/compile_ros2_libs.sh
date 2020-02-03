set -e
set -x

ROS2_INSTALL_LOCATION=$AMENT_PREFIX_PATH

OPTIMIZE=-O3

g++ $OPTIMIZE -Wall -shared -std=c++14 -fPIC `pkg-config --cflags python3` ros/ros2/cpp/node_init.cpp -o node_init`python3-config --extension-suffix` -I$ROS2_INSTALL_LOCATION/include -L$ROS2_INSTALL_LOCATION/lib -lrclcpp
g++ $OPTIMIZE -Wall -shared -std=c++14 -fPIC `pkg-config --cflags python3` ros/ros2/cpp/ros_bindings.cpp -o ros_bindings`python3-config --extension-suffix` -I$ROS2_INSTALL_LOCATION/include -L$ROS2_INSTALL_LOCATION/lib -lrclcpp -ltf2_ros -lurdf -lorocos-kdl -lkdl_parser
g++ $OPTIMIZE -Wall -shared -std=c++14 -fPIC `pkg-config --cflags python3` ros/ros2/cpp/messages/geometry_msgs/TwistCppImpl.cpp -o ros/ros2/cpp/messages/geometry_msgs/TwistCppImpl`python3-config --extension-suffix` -I$ROS2_INSTALL_LOCATION/include -L$ROS2_INSTALL_LOCATION/lib -lrclcpp -lgeometry_msgs__rosidl_typesupport_cpp
g++ $OPTIMIZE -Wall -shared -std=c++14 -fPIC `pkg-config --cflags python3` ros/ros2/cpp/messages/nav_msgs/OdometryCppImpl.cpp -o ros/ros2/cpp/messages/nav_msgs/OdometryCppImpl`python3-config --extension-suffix` -I$ROS2_INSTALL_LOCATION/include -L$ROS2_INSTALL_LOCATION/lib -lrclcpp -lnav_msgs__rosidl_typesupport_cpp
g++ $OPTIMIZE -Wall -shared -std=c++14 -fPIC `pkg-config --cflags python3` ros/ros2/cpp/messages/sensor_msgs/JointStateCppImpl.cpp -o ros/ros2/cpp/messages/sensor_msgs/JointStateCppImpl`python3-config --extension-suffix` -I$ROS2_INSTALL_LOCATION/include -L$ROS2_INSTALL_LOCATION/lib -lrclcpp -lsensor_msgs__rosidl_typesupport_cpp
