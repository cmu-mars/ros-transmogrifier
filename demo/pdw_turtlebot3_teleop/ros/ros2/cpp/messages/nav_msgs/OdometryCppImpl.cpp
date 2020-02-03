#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;

class OdometryCppImpl
{
private:
	rclcpp::Publisher<nav_msgs::msg::Odometry, std::allocator<void>>::SharedPtr publisher;

public:
	rclcpp::Publisher<nav_msgs::msg::Odometry, std::allocator<void>>::SharedPtr createPublisher(py::object node, std::string topic_name)
	{
		rclcpp::Node::SharedPtr n = node.cast<rclcpp::Node::SharedPtr>();
		publisher = n->create_publisher<nav_msgs::msg::Odometry>(topic_name, 100);
		return publisher;
	}
};

PYBIND11_MODULE(OdometryCppImpl, m) {
	py::class_<OdometryCppImpl>(m, "OdometryCppImpl")
		.def(py::init())
		.def("createPublisher", &OdometryCppImpl::createPublisher);
	py::class_<rclcpp::Publisher<nav_msgs::msg::Odometry, std::allocator<void>>,
			   rclcpp::Publisher<nav_msgs::msg::Odometry, std::allocator<void>>::SharedPtr>(m, "OdometryPublisher")
		.def("publish", (void (rclcpp::Publisher<nav_msgs::msg::Odometry, std::allocator<void>>::*)(const nav_msgs::msg::Odometry &)) &rclcpp::Publisher<nav_msgs::msg::Odometry, std::allocator<void>>::publish);
}
