#include <pybind11/pybind11.h>
#include <pybind11/operators.h>

#include <cassert>
//#include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
//#include <std_msgs/Header.h>
//#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

namespace py = pybind11;

// FIXME: It doesn't make sense to implement assert as a function.
void ros_assert(bool condition) {
	assert(condition);
}

void ros_error(std::string s) {
	//ROS_ERROR(s.c_str());
}

rclcpp::Time ros_time_now(rclcpp::Node::SharedPtr node)
{
	return node->now();
}

/*bool ros_ok(rclcpp::Node::SharedPtr node)
{
	return rclcpp::ok((rclcpp::Context::SharedPtr) node);
}*/

bool ros_ok()
{
	return rclcpp::ok();
}

void rclcpp_spin(rclcpp::Node::SharedPtr node)
{
	rclcpp::spin(node);
}

void rclcpp_spin_some(rclcpp::Node::SharedPtr node)
{
	rclcpp::spin_some(node);
}

/*namespace pybind11 {
	namespace detail {
		template <> struct type_caster<builtin_interfaces::msg::Time>
		{
		public:
			PYBIND11_TYPE_CASTER(builtin_interfaces::msg::Time, _("builtin_interfaces_msg_Time"));

			bool load(handle src, bool) {
				rclcpp::Time *rclcpp_time = src.cast<rclcpp::Time *>();

				// This will actually never get executed. If the cast fails,
				// cast_error will be thrown.
				if (!rclcpp_time) {
					return false;
				}

				value = *rclcpp_time;

				return true;
			}

			static handle cast(builtin_interfaces::msg::Time src, return_value_policy, handle) {
				rclcpp::Time rclcpp_time = src;
				return py::cast(rclcpp_time);
			}
		};
	}
}*/

PYBIND11_MODULE(ros_bindings, m) {
	//m.def("ros_ok", (bool (*)()) &rclcpp::ok);
	m.def("ros_ok", ros_ok);
	m.def("ros_spin", &rclcpp_spin);
	m.def("ros_spinSome", &rclcpp_spin_some);
	// FIXME: Naming this function assert doesn't work for now
	//        because assert is a keyword in Wyvern.
	m.def("ros_assert", &ros_assert);
	m.def("ros_time_now", &ros_time_now);

	//m.def("ros_error", &ros_error);

	py::class_<rclcpp::Duration>(m, "ros_Duration")
		//.def(py::init<int, int>())
		.def("toSec", &rclcpp::Duration::seconds);

	py::class_<rclcpp::Rate>(m, "ros_Rate")
		.def(py::init<double>())
		.def("sleep", &rclcpp::Rate::sleep); 

	py::class_<rclcpp::Time>(m, "ros_Time")
		.def(py::self - py::self)
		//.def(py::self + rclcpp::Duration())
		.def(py::self < py::self)
		.def(py::self > py::self);

	py::class_<builtin_interfaces::msg::Time, builtin_interfaces::msg::Time::SharedPtr>(m, "builtin_interfaces_msg_Time")
		.def(py::init<rclcpp::Time>());

	py::implicitly_convertible<rclcpp::Time, builtin_interfaces::msg::Time>();

	py::class_<geometry_msgs::msg::Twist, geometry_msgs::msg::Twist::SharedPtr>(m, "geometry_msgs_Twist")
		.def_readwrite("linear", &geometry_msgs::msg::Twist::linear)
		.def_readwrite("angular", &geometry_msgs::msg::Twist::angular);

	py::class_<geometry_msgs::msg::TransformStamped, geometry_msgs::msg::TransformStamped::SharedPtr>(m, "geometry_msgs_TransformStamped")
		.def(py::init())
		.def_readwrite("header", &geometry_msgs::msg::TransformStamped::header)
		.def_readwrite("child_frame_id", &geometry_msgs::msg::TransformStamped::child_frame_id)
		.def_readwrite("transform", &geometry_msgs::msg::TransformStamped::transform);

	py::class_<geometry_msgs::msg::Transform, geometry_msgs::msg::Transform::SharedPtr>(m, "geometry_msgs_Transform")
		.def_readwrite("translation", &geometry_msgs::msg::Transform::translation)
		.def_readwrite("rotation", &geometry_msgs::msg::Transform::rotation);

	py::class_<geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3::SharedPtr>(m, "geometry_msgs_Vector3")
		.def_readwrite("x", &geometry_msgs::msg::Vector3::x)
		.def_readwrite("y", &geometry_msgs::msg::Vector3::y)
		.def_readwrite("z", &geometry_msgs::msg::Vector3::z);

	py::class_<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState::SharedPtr>(m, "sensor_msgs_JointState")
		.def(py::init())
		.def_readwrite("header", &sensor_msgs::msg::JointState::header)
		.def_readwrite("name", &sensor_msgs::msg::JointState::name)
		.def_readwrite("position", &sensor_msgs::msg::JointState::position)
		.def_readwrite("velocity", &sensor_msgs::msg::JointState::velocity)
		.def_readwrite("effort", &sensor_msgs::msg::JointState::effort);

	py::class_<std_msgs::msg::Header, std_msgs::msg::Header::SharedPtr>(m, "std_msgs_Header")
		.def_readwrite("stamp", &std_msgs::msg::Header::stamp)
		.def_readwrite("frame_id", &std_msgs::msg::Header::frame_id);

	py::class_<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry::SharedPtr>(m, "nav_msgs_Odometry")
		.def(py::init())
		.def_readwrite("header", &nav_msgs::msg::Odometry::header)
		.def_readwrite("child_frame_id", &nav_msgs::msg::Odometry::child_frame_id)
		.def_readwrite("pose", &nav_msgs::msg::Odometry::pose)
		.def_readwrite("twist", &nav_msgs::msg::Odometry::twist);

	py::class_<geometry_msgs::msg::PoseWithCovariance, geometry_msgs::msg::PoseWithCovariance::SharedPtr>(m, "geometry_msgs_PoseWithCovariance")
		.def_readwrite("pose", &geometry_msgs::msg::PoseWithCovariance::pose)
		.def_readwrite("covariance", &geometry_msgs::msg::PoseWithCovariance::covariance);

	py::class_<geometry_msgs::msg::Pose, geometry_msgs::msg::Pose::SharedPtr>(m, "geometry_msgs_Pose")
		.def_readwrite("position", &geometry_msgs::msg::Pose::position)
		.def_readwrite("orientation", &geometry_msgs::msg::Pose::orientation);

	py::class_<geometry_msgs::msg::Point, geometry_msgs::msg::Point::SharedPtr>(m, "geometry_msgs::Point")
		.def_readwrite("x", &geometry_msgs::msg::Point::x)
		.def_readwrite("y", &geometry_msgs::msg::Point::y)
		.def_readwrite("z", &geometry_msgs::msg::Point::z);

	py::class_<geometry_msgs::msg::Quaternion, geometry_msgs::msg::Quaternion::SharedPtr>(m, "geometry_msgs::Quaternion")
		.def_readwrite("x", &geometry_msgs::msg::Quaternion::x)
		.def_readwrite("y", &geometry_msgs::msg::Quaternion::y)
		.def_readwrite("z", &geometry_msgs::msg::Quaternion::z)
		.def_readwrite("w", &geometry_msgs::msg::Quaternion::w);

	py::class_<geometry_msgs::msg::TwistWithCovariance, geometry_msgs::msg::TwistWithCovariance::SharedPtr>(m, "geometry_msgs_TwistWithCovariance")
		.def_readwrite("twist", &geometry_msgs::msg::TwistWithCovariance::twist)
		.def_readwrite("covariance", &geometry_msgs::msg::TwistWithCovariance::covariance);

	//py::class_<tf2_msgs::TFMessage>(m, "tf2_msgs_TFMessage");

	py::class_<tf2_ros::StaticTransformBroadcaster>(m, "tf2_ros_StaticTransformBroadcaster")
		.def(py::init<rclcpp::Node::SharedPtr>())
		.def("sendTransform", (void (tf2_ros::StaticTransformBroadcaster::*)(const geometry_msgs::msg::TransformStamped &)) &tf2_ros::StaticTransformBroadcaster::sendTransform)
		.def("sendTransform", (void (tf2_ros::StaticTransformBroadcaster::*)(const std::vector<geometry_msgs::msg::TransformStamped> &)) &tf2_ros::StaticTransformBroadcaster::sendTransform);

	py::class_<tf2_ros::TransformBroadcaster>(m, "tf2_ros_TransformBroadcaster")
		.def(py::init<rclcpp::Node::SharedPtr>())
		.def("sendTransform", (void (tf2_ros::TransformBroadcaster::*)(const geometry_msgs::msg::TransformStamped &)) &tf2_ros::TransformBroadcaster::sendTransform)
		.def("sendTransform", (void (tf2_ros::TransformBroadcaster::*)(const std::vector<geometry_msgs::msg::TransformStamped> &)) &tf2_ros::TransformBroadcaster::sendTransform);

	py::class_<std::array<double, 36>>(m, "std_array_double_36", py::buffer_protocol())
		.def_buffer([](std::array<double, 36> &arr) -> py::buffer_info {
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

	py::class_<std::vector<geometry_msgs::msg::TransformStamped>>(m, "std_vector_geometry_msgs_TransformStamped")
		.def(py::init())
		.def("add", (void (std::vector<geometry_msgs::msg::TransformStamped>::*)(const geometry_msgs::msg::TransformStamped &))
						  &std::vector<geometry_msgs::msg::TransformStamped>::push_back);

	py::class_<urdf::ModelInterface>(m, "urdf_ModelInterface");

	py::class_<urdf::Model, urdf::ModelInterface>(m, "urdf_Model")
		.def(py::init())
		.def("initFile", &urdf::Model::initFile);

	py::class_<KDL::Tree>(m, "KDL_Tree")
		.def(py::init());

	m.def("kdl_parser_treeFromUrdfModel", &kdl_parser::treeFromUrdfModel);
}
