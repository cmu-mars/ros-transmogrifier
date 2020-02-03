#include <pybind11/pybind11.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace py = pybind11;

ros::NodeHandle init(const std::string &node_name)
{
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, node_name);
	ros::NodeHandle n;
	return n;
}

PYBIND11_MODULE(node_init, m) {
	m.def("init", &init);
	py::class_<ros::NodeHandle>(m, "ros_NodeHandle");
}
