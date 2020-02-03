#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <pybind11/pybind11.h>

namespace py = pybind11;

class OdometryPublisher : public ros::Publisher
{
public:
	OdometryPublisher() { }

	OdometryPublisher(ros::Publisher publisher)
	: ros::Publisher(publisher)
	{ }
};

class OdometryCppImpl
{
private:
	OdometryPublisher publisher;

public:
	OdometryPublisher createPublisher(py::object node_handle, std::string topic_name)
	{
		ros::NodeHandle nh = node_handle.cast<ros::NodeHandle>();
		publisher = nh.advertise<nav_msgs::Odometry>(topic_name, 100);
		return publisher;
	}
};

PYBIND11_MODULE(OdometryCppImpl, m) {
	py::class_<OdometryCppImpl>(m, "OdometryCppImpl")
		.def(py::init())
		.def("createPublisher", &OdometryCppImpl::createPublisher);
	py::class_<OdometryPublisher>(m, "ros_Publisher_nav_msgs_Odometry")
		.def("publish", (void (ros::Publisher::*)(const nav_msgs::Odometry &) const) &ros::Publisher::publish<nav_msgs::Odometry>);
}
