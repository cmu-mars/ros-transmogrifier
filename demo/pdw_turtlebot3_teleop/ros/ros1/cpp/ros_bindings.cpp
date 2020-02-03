#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace py = pybind11;

void ros_assert(bool condition) {
	ROS_ASSERT(condition);
}

void ros_error(std::string s) {
	ROS_ERROR(s.c_str());
}

PYBIND11_MODULE(ros_bindings, m) {
	m.def("ros_ok", &ros::ok);
	m.def("ros_spin", (void (*)()) &ros::spin);
	m.def("ros_spinSome", &ros::spinOnce);
	// FIXME: Naming this function assert doesn't work for now
	//        because assert is a keyword in Wyvern.
	m.def("ros_assert", &ros_assert);
	m.def("ros_time_now", &ros::Time::now);

	m.def("ros_error", &ros_error);

	py::class_<ros::Duration>(m, "ros_Duration")
		.def(py::init<int, int>())
		.def("toSec", &ros::Duration::toSec);

	py::class_<ros::Rate>(m, "ros_Rate")
		.def(py::init<double>())
		.def("sleep", &ros::Rate::sleep);

	py::class_<ros::Time>(m, "ros_Time")
		.def(py::self - py::self)
		.def(py::self + ros::Duration())
		.def(py::self < py::self)
		.def(py::self > py::self);

	py::class_<geometry_msgs::TransformStamped>(m, "geometry_msgs_TransformStamped")
		.def(py::init())
		.def_readwrite("header", &geometry_msgs::TransformStamped::header)
		.def_readwrite("child_frame_id", &geometry_msgs::TransformStamped::child_frame_id)
		.def_readwrite("transform", &geometry_msgs::TransformStamped::transform);

	py::class_<geometry_msgs::Transform>(m, "geometry_msgs_Transform")
		.def_readwrite("translation", &geometry_msgs::Transform::translation)
		.def_readwrite("rotation", &geometry_msgs::Transform::rotation);

	py::class_<geometry_msgs::Vector3>(m, "geometry_msgs_Vector3")
		.def_readwrite("x", &geometry_msgs::Vector3::x)
		.def_readwrite("y", &geometry_msgs::Vector3::y)
		.def_readwrite("z", &geometry_msgs::Vector3::z);

	py::class_<sensor_msgs::JointState>(m, "sensor_msgs_JointState")
		.def(py::init())
		.def_readwrite("header", &sensor_msgs::JointState::header)
		.def_readwrite("name", &sensor_msgs::JointState::name)
		.def_readwrite("position", &sensor_msgs::JointState::position)
		.def_readwrite("velocity", &sensor_msgs::JointState::velocity)
		.def_readwrite("effort", &sensor_msgs::JointState::effort);

	py::class_<std_msgs::Header>(m, "std_msgs_Header")
		.def_readwrite("stamp", &std_msgs::Header::stamp)
		.def_readwrite("frame_id", &std_msgs::Header::frame_id);

	py::class_<nav_msgs::Odometry>(m, "nav_msgs_Odometry")
		.def(py::init())
		.def_readwrite("header", &nav_msgs::Odometry::header)
		.def_readwrite("child_frame_id", &nav_msgs::Odometry::child_frame_id)
		.def_readwrite("pose", &nav_msgs::Odometry::pose)
		.def_readwrite("twist", &nav_msgs::Odometry::twist);

	py::class_<geometry_msgs::PoseWithCovariance>(m, "geometry_msgs_PoseWithCovariance")
		.def_readwrite("pose", &geometry_msgs::PoseWithCovariance::pose)
		.def_readwrite("covariance", &geometry_msgs::PoseWithCovariance::covariance);

	py::class_<geometry_msgs::Pose>(m, "geometry_msgs_Pose")
		.def_readwrite("position", &geometry_msgs::Pose::position)
		.def_readwrite("orientation", &geometry_msgs::Pose::orientation);

	py::class_<geometry_msgs::Point>(m, "geometry_msgs::Point")
		.def_readwrite("x", &geometry_msgs::Point::x)
		.def_readwrite("y", &geometry_msgs::Point::y)
		.def_readwrite("z", &geometry_msgs::Point::z);

	py::class_<geometry_msgs::Quaternion>(m, "geometry_msgs::Quaternion")
		.def_readwrite("x", &geometry_msgs::Quaternion::x)
		.def_readwrite("y", &geometry_msgs::Quaternion::y)
		.def_readwrite("z", &geometry_msgs::Quaternion::z)
		.def_readwrite("w", &geometry_msgs::Quaternion::w);

	py::class_<geometry_msgs::TwistWithCovariance>(m, "geometry_msgs_TwistWithCovariance")
		.def_readwrite("twist", &geometry_msgs::TwistWithCovariance::twist)
		.def_readwrite("covariance", &geometry_msgs::TwistWithCovariance::covariance);

	py::class_<tf2_msgs::TFMessage>(m, "tf2_msgs_TFMessage");

	py::class_<tf2_ros::StaticTransformBroadcaster>(m, "tf2_ros_StaticTransformBroadcaster")
		.def(py::init())
		.def("sendTransform", (void (tf2_ros::StaticTransformBroadcaster::*)(const geometry_msgs::TransformStamped &)) &tf2_ros::StaticTransformBroadcaster::sendTransform)
		.def("sendTransform", (void (tf2_ros::StaticTransformBroadcaster::*)(const std::vector<geometry_msgs::TransformStamped> &)) &tf2_ros::StaticTransformBroadcaster::sendTransform);

	py::class_<tf2_ros::TransformBroadcaster>(m, "tf2_ros_TransformBroadcaster")
		.def(py::init())
		.def("sendTransform", (void (tf2_ros::TransformBroadcaster::*)(const geometry_msgs::TransformStamped &)) &tf2_ros::TransformBroadcaster::sendTransform)
		.def("sendTransform", (void (tf2_ros::TransformBroadcaster::*)(const std::vector<geometry_msgs::TransformStamped> &)) &tf2_ros::TransformBroadcaster::sendTransform);

	py::class_<boost::array<double, 36>>(m, "boost_array_double_36", py::buffer_protocol())
		.def_buffer([](boost::array<double, 36> &arr) -> py::buffer_info {
			return py::buffer_info(
				arr.data(),
				sizeof(double),
				py::format_descriptor<double>::format(),
				1,
				{ 36 },
				{ sizeof(double) }
			);
		});

	py::class_<std::vector<std::string>>(m, "std_vector_std_string")
		.def("push_back", (void (std::vector<std::string>::*)(const std::string &)) &std::vector<std::string>::push_back)
		.def("size", &std::vector<std::string>::size)
		.def("__getitem__", [](const std::vector<std::string> &v, size_t i) { return v[i]; })
		.def("__setitem__", [](std::vector<std::string> &v, size_t i, std::string val) { v[i] = val; });

	py::class_<std::vector<double>>(m, "std_vector_double")
		.def("resize", (void (std::vector<double>::*)(size_t, const double &)) &std::vector<double>::resize)
		.def("size", &std::vector<double>::size)
		.def("__getitem__", [](const std::vector<double> &v, size_t i) { return v[i]; })
		.def("__setitem__", [](std::vector<double> &v, size_t i, double val) { v[i] = val; });

	py::class_<std::vector<geometry_msgs::TransformStamped>>(m, "std_vector_geometry_msgs_TransformStamped")
		.def(py::init())
		.def("add", (void (std::vector<geometry_msgs::TransformStamped>::*)(const geometry_msgs::TransformStamped &))
						  &std::vector<geometry_msgs::TransformStamped>::push_back);

	py::class_<urdf::ModelInterface>(m, "urdf_ModelInterface");

	py::class_<urdf::Model, urdf::ModelInterface>(m, "urdf_Model")
		.def(py::init())
		.def("initFile", &urdf::Model::initFile);

	py::class_<KDL::Tree>(m, "KDL_Tree")
		.def(py::init());

	m.def("kdl_parser_treeFromUrdfModel", &kdl_parser::treeFromUrdfModel);
}
