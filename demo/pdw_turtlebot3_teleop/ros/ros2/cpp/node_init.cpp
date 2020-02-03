#include <pybind11/pybind11.h>

#include <rclcpp/rclcpp.hpp>

#include <csignal>

namespace py = pybind11;

void sigint_handler(int parameter) { }

rclcpp::Node::SharedPtr init(const std::string &node_name)
{
	// FIXME: This is a workaround to https://github.com/eProsima/Fast-RTPS/issues/235
	// which causes the program to get stuck instead of exiting when Ctrl+C is
	// pressed. Is it safe? Probably not, but it doesn't matter for this example.
	signal(SIGINT, sigint_handler);

	int argc = 0;
	char **argv = NULL;
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared(node_name);
	return n;
}

PYBIND11_MODULE(node_init, m) {
	m.def("init", &init);
	py::class_<rclcpp::Node, rclcpp::Node::SharedPtr>(m, "rclcpp_Node");
}
