#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;

// This is a hack to avoid the "type is already registered"
// runtime error that occurs when ros::Publisher is registered
// more than once using py::class_.
class JointStatePublisher : public ros::Publisher
{
public:
	JointStatePublisher() { }

	JointStatePublisher(ros::Publisher publisher)
	: ros::Publisher(publisher)
	{ }
};

class JointStateSubscriber : public ros::Subscriber
{
public:
	JointStateSubscriber() { }

	JointStateSubscriber(ros::Subscriber subscriber)
	: ros::Subscriber(subscriber)
	{ }
};

class JointStateCppImpl
{
private:
	py::object wyv_callback;
	JointStatePublisher publisher;
	JointStateSubscriber subscriber;

public:
	JointStatePublisher createPublisher(py::object node_handle, std::string topic_name)
	{
		ros::NodeHandle nh = node_handle.cast<ros::NodeHandle>();
		publisher = nh.advertise<sensor_msgs::JointState>(topic_name, 100);
		return publisher;
	}

	JointStateSubscriber createSubscriber(py::object node_handle, std::string topic_name, py::object callback)
	{
		this->wyv_callback = callback.attr("apply");
		ros::NodeHandle nh = node_handle.cast<ros::NodeHandle>();
		subscriber = nh.subscribe(topic_name, 100, &JointStateCppImpl::callback, this);
		return subscriber;
	}

	void callback(const sensor_msgs::JointState msg)
	{
		wyv_callback(msg);
	}
};

PYBIND11_MODULE(JointStateCppImpl, m) {
	py::class_<JointStateCppImpl>(m, "JointStateCppImpl")
		.def(py::init())
		.def("createPublisher", &JointStateCppImpl::createPublisher)
		.def("createSubscriber", &JointStateCppImpl::createSubscriber);
	py::class_<JointStatePublisher>(m, "JointStatePublisher")
		.def("publish", (void (ros::Publisher::*)(const sensor_msgs::JointState &) const) &ros::Publisher::publish<sensor_msgs::JointState>);
	py::class_<JointStateSubscriber>(m, "JointStateSubscriber");
}
