#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;

class TwistSubscriber : public ros::Subscriber
{
public:
	TwistSubscriber() { }

	TwistSubscriber(ros::Subscriber subscriber)
	: ros::Subscriber(subscriber)
	{ }
};

class TwistCppImpl
{
private:
	py::object wyv_callback;
	TwistSubscriber subscriber;

public:
	TwistSubscriber createSubscriber(py::object node_handle, std::string topic_name, py::object callback)
	{
		this->wyv_callback = callback.attr("apply");
		ros::NodeHandle nh = node_handle.cast<ros::NodeHandle>();
		subscriber = nh.subscribe(topic_name, 100, &TwistCppImpl::callback, this);
		return subscriber;
	}

	void callback(const geometry_msgs::Twist msg)
	{
		wyv_callback(msg);
	}
};

PYBIND11_MODULE(TwistCppImpl, m) {
	py::class_<TwistCppImpl>(m, "TwistCppImpl")
		.def(py::init())
		.def("createSubscriber", &TwistCppImpl::createSubscriber);
	py::class_<TwistSubscriber>(m, "TwistSubscriber");
	py::class_<geometry_msgs::Twist>(m, "geometry_msgs_Twist")
		.def_readwrite("linear", &geometry_msgs::Twist::linear)
		.def_readwrite("angular", &geometry_msgs::Twist::angular);
	/*py::class_<geometry_msgs::Vector3>(m, "geometry_msgs_Vector3")
		.def_readwrite("x", &geometry_msgs::Vector3::x)
		.def_readwrite("y", &geometry_msgs::Vector3::y)
		.def_readwrite("z", &geometry_msgs::Vector3::z);*/
}
