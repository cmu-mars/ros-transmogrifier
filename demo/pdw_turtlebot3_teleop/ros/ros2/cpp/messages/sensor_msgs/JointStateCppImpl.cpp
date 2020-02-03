#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <pybind11/pybind11.h>

namespace py = pybind11;

class JointStateCppImpl
{
private:
	py::object wyv_callback;
	rclcpp::Publisher<sensor_msgs::msg::JointState, std::allocator<void>>::SharedPtr publisher;
	rclcpp::Subscription<sensor_msgs::msg::JointState, std::allocator<void>>::SharedPtr subscriber;

public:
	rclcpp::Publisher<sensor_msgs::msg::JointState, std::allocator<void>>::SharedPtr createPublisher(py::object node, std::string topic_name)
	{
		rclcpp::Node::SharedPtr n = node.cast<rclcpp::Node::SharedPtr>();
		publisher = n->create_publisher<sensor_msgs::msg::JointState>(topic_name, 100);
		return publisher;
	}

	rclcpp::Subscription<sensor_msgs::msg::JointState, std::allocator<void>>::SharedPtr createSubscriber(py::object node, std::string topic_name, py::object callback)
	{
		this->wyv_callback = callback.attr("apply");
		rclcpp::Node::SharedPtr n = node.cast<rclcpp::Node::SharedPtr>();

		auto qos = rclcpp::QoS(rclcpp::KeepLast(100));
		subscriber = n->create_subscription<sensor_msgs::msg::JointState>(topic_name, qos, [this](const sensor_msgs::msg::JointState::SharedPtr msg) { wyv_callback(msg); });
		return subscriber;
	}
};

PYBIND11_MODULE(JointStateCppImpl, m) {
	py::class_<JointStateCppImpl>(m, "JointStateCppImpl")
		.def(py::init())
		.def("createPublisher", &JointStateCppImpl::createPublisher)
		.def("createSubscriber", &JointStateCppImpl::createSubscriber);
	py::class_<rclcpp::Publisher<sensor_msgs::msg::JointState, std::allocator<void>>,
			   rclcpp::Publisher<sensor_msgs::msg::JointState, std::allocator<void>>::SharedPtr>(m, "JointStatePublisher")
		.def("publish", (void (rclcpp::Publisher<sensor_msgs::msg::JointState, std::allocator<void>>::*)(const sensor_msgs::msg::JointState &)) &rclcpp::Publisher<sensor_msgs::msg::JointState, std::allocator<void>>::publish);
	py::class_<rclcpp::Subscription<sensor_msgs::msg::JointState, std::allocator<void>>,
			   rclcpp::Subscription<sensor_msgs::msg::JointState, std::allocator<void>>::SharedPtr>(m, "JointStateSubscriber");
}
