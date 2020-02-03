#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;

class TwistCppImpl
{
private:
	py::object wyv_callback;
	rclcpp::Subscription<geometry_msgs::msg::Twist, std::allocator<void>>::SharedPtr subscriber;

public:
	rclcpp::Subscription<geometry_msgs::msg::Twist, std::allocator<void>>::SharedPtr createSubscriber(py::object node, std::string topic_name, py::object callback)
	{
		this->wyv_callback = callback.attr("apply");
		rclcpp::Node::SharedPtr n = node.cast<rclcpp::Node::SharedPtr>();

		auto qos = rclcpp::QoS(rclcpp::KeepLast(100));
		subscriber = n->create_subscription<geometry_msgs::msg::Twist>(topic_name, qos, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { wyv_callback(msg); });
		return subscriber;
	}
};

PYBIND11_MODULE(TwistCppImpl, m) {
	py::class_<TwistCppImpl>(m, "TwistCppImpl")
		.def(py::init())
		.def("createSubscriber", &TwistCppImpl::createSubscriber);
	py::class_<rclcpp::Subscription<geometry_msgs::msg::Twist, std::allocator<void>>,
			   rclcpp::Subscription<geometry_msgs::msg::Twist, std::allocator<void>>::SharedPtr>(m, "TwistSubscriber");
}
