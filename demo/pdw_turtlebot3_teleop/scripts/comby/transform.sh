TURTLEBOT3FAKENODE_CPP=pdw_turtlebot3_fake.cpp
TURTLEBOT3FAKENODE_H=pdw_turtlebot3_fake.h

JOINTSTATELISTENER_CPP=pdw_joint_state_listener.cpp
JOINTSTATELISTENER_H=pdw_joint_state_listener.h

ROBOTSTATEPUBLISHER_CPP=pdw_robot_state_publisher.cpp
ROBOTSTATEPUBLISHER_H=pdw_robot_state_publisher.h

cd cpp/pdw_turtlebot3_fake/include/pdw_turtlebot3_fake

COMBY_M=$(cat <<MATCH
#include <:[1]>
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
class Turtlebot3Fake {:[1]Turtlebot3Fake();:[2]}
MATCH
)

COMBY_R=$(cat <<REWRITE
#include <pybind11/pybind11.h>

namespace py = pybind11;

class Turtlebot3Fake
{:[1]Turtlebot3Fake(py::object joint_states_pub, py::object odom_pub, py::object tf_pub, py::object ros);:[2]
 private:
  py::object joint_states_pub;
  py::object odom_pub;
  py::object tf_pub;
  py::object ros;
  friend void commandVelocityCallback(py::object py_fake_turtlebot3, py::object cmd_vel_msg);
}
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
sensor_msgs::JointState :[[1]]
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1]
REWRITE
)

# This is okay because the C++/Python interop is a workaround to us not having a C++ backend for Wyvern.
# If we did have a C++ backend, then we would have stronger typing for messages.
comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
nav_msgs::Odometry :[[1]]
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1]
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
ros::Publisher :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
ros::Subscriber :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
ros::NodeHandle :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
ros::Time :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1];
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
geometry_msgs::TwistConstPtr :[[1]]
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1]
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
(ros::Duration :[[1]])
MATCH
)

COMBY_R=$(cat <<REWRITE
(py::object :[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
#include :[1]
MATCH
)

COMBY_R=$(cat <<REWRITE
#include <tf2/LinearMath/Quaternion.h>
#include :[1]
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

# FIXME: Pattern repeated from .cpp
#        All these patterns should be conflated into one script that
#        should be able to handle any input C++ header or source file.
COMBY_M=$(cat <<MATCH
(geometry_msgs::TransformStamped& :[1])
MATCH
)

COMBY_R=$(cat <<REWRITE
(py::object& :[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
tf::TransformBroadcaster :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

COMBY_M=$(cat <<MATCH
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_H" -in-place

cd ../../src

comby 'int main(:[1]) { :[2] }' '' "$TURTLEBOT3FAKENODE_CPP" -in-place

#TODO: Have to figure out how to handle cases when the constructor might have arguments.
#SIN: Semantic information needed
#     To properly handle NodeHandles and messages, we would need type information.
COMBY_M=$(cat <<MATCH
:[[1]]:::[1]() : n:[3]("~") {:[4]}
MATCH
)

COMBY_R=$(cat <<REWRITE
:[1]:::[1](py::object joint_states_pub, py::object odom_pub, py::object tf_pub, py::object ros)
: joint_states_(ros.attr("messages").attr("sensor_msgs").attr("JointState")()),
  odom_(ros.attr("messages").attr("nav_msgs").attr("Odometry")()),
  joint_states_pub(joint_states_pub), odom_pub(odom_pub), tf_pub(tf_pub), ros(ros)
{:[4]}
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
ROS_ASSERT(:[1])
MATCH
)

COMBY_R=$(cat <<REWRITE
ros.attr("_assert")(:[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

#SIN
COMBY_M=$(cat <<MATCH
=:[1].param<:[2]>("tb3_model", :[3])
MATCH
)

COMBY_R=$(cat <<REWRITE
= "burger"
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

#SIN
COMBY_M=$(cat <<MATCH
:[ 1]:[[2]].param(:[3], :[4], :[5])
MATCH
)

COMBY_R=$(cat <<REWRITE
:[ 1]:[4] = :[5]
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

#SIN
COMBY_M=$(cat <<MATCH
joint_states_.:[[1]].:[[2]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
joint_states_.attr(":[1]").attr(":[2]") =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
odom_.:[[1]].:[[2]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
odom_.attr(":[1]").attr(":[2]") =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
odom_.:[[1]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
odom_.attr(":[1]") =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
memcpy(&(odom_.:[[1]].covariance):[2]);
MATCH
)

COMBY_R=$(cat <<REWRITE
double *odom__:[1]_covariance = (double *)((py::buffer)odom_.attr(":[1]").attr("covariance")).request().ptr;
  memcpy(odom__:[1]_covariance:[2]);
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
joint_states_.:[[1]].:[[2]](:[3])
MATCH
)

COMBY_R=$(cat <<REWRITE
joint_states_.attr(":[1]").attr(":[2]")(:[3])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
:[[1]] = n:[[2]].advertise<:[3]>(:[4]);
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
:[[1]] = n:[[2]].subscribe(:[3]);
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
ros::Time::now()
MATCH
)

COMBY_R=$(cat <<REWRITE
ros.attr("time").attr("now")()
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
geometry_msgs::TwistConstPtr :[[1]]
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1]
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
= cmd_vel_msg->linear.x
MATCH
)

COMBY_R=$(cat <<REWRITE
= cmd_vel_msg.attr("linear").attr("x").cast<double>()
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
= cmd_vel_msg->angular.z
MATCH
)

COMBY_R=$(cat <<REWRITE
= cmd_vel_msg.attr("angular").attr("z").cast<double>()
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
ros::Time :[[1]] 
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1] 
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
ros::Duration :[[1]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1] =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
.toSec()
MATCH
)

COMBY_R=$(cat <<REWRITE
.attr("toSec")().cast<double>()
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
(ros::Duration :[[1]])
MATCH
)

COMBY_R=$(cat <<REWRITE
(py::object :[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

# This transformation, if done right, should be handled by the synthesis part rather than
# done here.
# 
# Here, I tried to use the more general pattern :[[1]]:[2] = tf::createQuaternionMsgFromYaw(:[3])
# but I got a "Timeout for input" error from comby. So, I have used this pattern for the time being.
COMBY_M=$(cat <<MATCH
odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(:[1]);
MATCH
)

COMBY_R=$(cat <<REWRITE

  tf2::Quaternion q;
  q.setRPY(0, 0, :[1]);

  odom_.pose.pose.orientation.x = q.x();
  odom_.pose.pose.orientation.y = q.y();
  odom_.pose.pose.orientation.z = q.z();
  odom_.pose.pose.orientation.w = q.w();
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

# SIN
COMBY_M=$(cat <<MATCH
odom_.:[[1]].:[[2]].:[[3]].:[[4]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
odom_.attr(":[1]").attr(":[2]").attr(":[3]").attr(":[4]") =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
odom_pub_.publish(:[1])
MATCH
)

COMBY_R=$(cat <<REWRITE
odom_pub.attr("getMsgConsumed")(:[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
geometry_msgs::TransformStamped :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1] = ros.attr("messages").attr("geometry_msgs").attr("TransformStamped")();
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

# This is not general. What if geometry_msgs::TransformStamped appears at another
# position in the argument list? And what if it is not a reference?
COMBY_M=$(cat <<MATCH
(geometry_msgs::TransformStamped& :[1])
MATCH
)

COMBY_R=$(cat <<REWRITE
(py::object& :[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

# SIN
COMBY_M=$(cat <<MATCH
odom_tf.:[[1]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
odom_tf.attr(":[1]") =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
odom_tf.:[[1]].:[[2]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
odom_tf.attr(":[1]").attr(":[2]") =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
odom_tf.:[[1]].:[[2]].:[[3]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
odom_tf.attr(":[1]").attr(":[2]").attr(":[3]") =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
= odom_.:[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
= odom_.attr(":[1]");
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
= odom_.:[[1]].:[[2]].:[[3]];
MATCH
)

COMBY_R=$(cat <<REWRITE
= odom_.attr(":[1]").attr(":[2]").attr(":[3]");
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
= odom_.:[[1]].:[[2]].:[[3]].:[[4]];
MATCH
)

COMBY_R=$(cat <<REWRITE
= odom_.attr(":[1]").attr(":[2]").attr(":[3]").attr(":[4]");
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
tf_broadcaster_.sendTransform(:[1])
MATCH
)

COMBY_R=$(cat <<REWRITE
tf_pub.attr("getTfConsumed")(:[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

# FIXME: This won't work if the statement is a comma-separated
#        list of assignments.
COMBY_M=$(cat <<MATCH
joint_states_.:[[1]][:[2]] = :[3];
MATCH
)

COMBY_R=$(cat <<REWRITE
joint_states_.attr(":[1]").attr("__setitem__")(:[2], :[3]);
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
joint_states_pub_.publish(:[1])
MATCH
)

COMBY_R=$(cat <<REWRITE
joint_states_pub.attr("getMsgConsumed")(:[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
:[1]
MATCH
)

COMBY_R=$(cat <<REWRITE
:[1]
void commandVelocityCallback(py::object py_fake_turtlebot3, py::object cmd_vel_msg)
{
  Turtlebot3Fake *fake_turtlebot3 = py_fake_turtlebot3.cast<Turtlebot3Fake *>();
  fake_turtlebot3->commandVelocityCallback(cmd_vel_msg);
}

PYBIND11_MODULE(CppTurtleBot3Fake, m) {
  m.def("commandVelocityCallback", &commandVelocityCallback);
  py::class_<Turtlebot3Fake>(m, "Turtlebot3Fake")
    .def(py::init<py::object, py::object, py::object, py::object>())
    .def("update", &Turtlebot3Fake::update);
}
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

COMBY_M=$(cat <<MATCH
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$TURTLEBOT3FAKENODE_CPP" -in-place

cd ../../pdw_robot_state_publisher/include/pdw_robot_state_publisher

COMBY_M=$(cat <<MATCH
#include:[1]#include <urdf/model.h>
#include <kdl/tree.hpp>
MATCH
)

COMBY_R=$(cat <<REWRITE
#include <map>
#include <urdf/model.h>
#include <kdl/tree.hpp>
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
using namespace ros;
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
typedef boost:::[1]:[[2]];
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
JointStateListener(:[1], const MimicMap& m, :[2]);
MATCH
)

COMBY_R=$(cat <<REWRITE
JointStateListener(py::object tf_pub, py::object tf_static_pub, py::object ros, :[1], :[2]);
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
namespace :[[1]] { class :[[2]] {:[3]}; }
MATCH
)

COMBY_R=$(cat <<REWRITE
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace :[1] {

class :[2] {:[3]
private:
  py::object tf_pub;
  py::object tf_static_pub;
  py::object ros;
};
}
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
Duration :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1];
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
(const JointStateConstPtr& :[[1]])
MATCH
)

COMBY_R=$(cat <<REWRITE
(py::object :[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
ros::Time :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1];
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
, ros::Time>
MATCH
)

COMBY_R=$(cat <<REWRITE
, py::object>
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
Subscriber :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

COMBY_M=$(cat <<MATCH
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_H" -in-place

cd ../../src

COMBY_M=$(cat <<MATCH
#include <:[1]>
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
using namespace ros;
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

# This rewriting is specific to this example. This is okay because
# we are not producing a fully automated tool. Some effort will
# be required to rewrite nodes in Wyvern in order to get the benefit
# of the ease of changing ROS connectors.
COMBY_M=$(cat <<MATCH
JointStateListener::JointStateListener(:[1], const MimicMap& m, :[2]) : state_publisher_(:[3]), mimic_(m) {:[4]}
MATCH
)

# The port object arguments are added first here as opposed to last
# because the JointStateListener constructor has default arguments
# which causes the compiler to complain because we don't supply
# default values to the port object arguments. So, we should perhaps
# always add port object arguments in the front.
COMBY_R=$(cat <<REWRITE
JointStateListener::JointStateListener(py::object tf_pub, py::object tf_static_pub, py::object ros, :[1], :[2])
  : state_publisher_(tf_pub, tf_static_pub, ros, :[3]), tf_pub(tf_pub), tf_static_pub(tf_static_pub), ros(ros)
{
  for(std::map< std::string, urdf::JointSharedPtr >::const_iterator i = model.joints_.begin(); i != model.joints_.end(); i++) {
    if(i->second->mimic) {
      mimic_.insert(make_pair(i->first, i->second->mimic));
    }
  }:[4]}
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
int main(:[1]) {:[2]}
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
ros::NodeHandle :[1];
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
:[[1]].param(:[2], :[3], :[4]);
MATCH
)

COMBY_R=$(cat <<REWRITE
:[3] = :[4];
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
:[[1]].searchParam(:[2]);
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

#COMBY_M=$(cat <<MATCH
#= ros::Duration(:[1]);
#MATCH
#)

#COMBY_R=$(cat <<REWRITE
#= ros.attr("createDurationFromSeconds")(:[1]);
#REWRITE
#)

#comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
ros::TransportHints :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
:[[1]].tcpNoDelay(:[2]);
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
(const JointStateConstPtr& state)
MATCH
)

COMBY_R=$(cat <<REWRITE
(py::object state)
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
:[[1]] = :[[2]].subscribe(:[3]);
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
state->:[[1]].:[[2]]() !=
MATCH
)

COMBY_R=$(cat <<REWRITE
state.attr(":[1]").attr(":[2]")() !=
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
!= state->:[[1]].:[[2]]()
MATCH
)

COMBY_R=$(cat <<REWRITE
!= state.attr(":[1]").attr(":[2]")()
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
<:[1]state->:[[2]].:[[3]]();
MATCH
)

COMBY_R=$(cat <<REWRITE
<:[1]state.attr(":[2]").attr(":[3]")().cast<size_t>();
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
state->:[[1]][:[2]].c_str()
MATCH
)

COMBY_R=$(cat <<REWRITE
state.attr(":[1]").attr("__getitem__")(:[2]).cast<std::string>().c_str()
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
ROS_WARN_THROTTLE(:[1],:[2])
MATCH
)

COMBY_R=$(cat <<REWRITE
fprintf(stderr,:[2])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
ROS_ERROR(:[1])
MATCH
)

COMBY_R=$(cat <<REWRITE
fprintf(stderr, :[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
ros::Time :[[1]]
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1]
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
ros::Time::now()
MATCH
)

COMBY_R=$(cat <<REWRITE
ros.attr("time").attr("now")()
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

# This would not work properly if multiple arguments are passed to ROS_WARN.
COMBY_M=$(cat <<MATCH
ROS_WARN(":[1]")
MATCH
)

COMBY_R=$(cat <<REWRITE
fprintf(stderr, ":[1]""\n")
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
[state->:[[1]][:[2]]]
MATCH
)

COMBY_R=$(cat <<REWRITE
[state.attr(":[1]").attr("__getitem__")(:[2]).cast<std::string>()]
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
state->name[:[1]],
MATCH
)

COMBY_R=$(cat <<REWRITE
state.attr("name").attr("__getitem__")(:[1]).cast<std::string>(),
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
, state->position[:[1]]
MATCH
)

COMBY_R=$(cat <<REWRITE
, state.attr("position").attr("__getitem__")(:[1]).cast<double>()
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
= state->:[[1]].:[[2]];
MATCH
)

COMBY_R=$(cat <<REWRITE
= state.attr(":[1]").attr(":[2]");
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
 state->:[[1]].:[[2]],
MATCH
)

COMBY_R=$(cat <<REWRITE
 state.attr(":[1]").attr(":[2]"),
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
(state->:[[1]].:[[2]]())
MATCH
)

COMBY_R=$(cat <<REWRITE
(state.attr(":[1]").attr(":[2]")())
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

COMBY_M=$(cat <<MATCH
:[1]
MATCH
)

COMBY_R=$(cat <<REWRITE
:[1]class JointStateListenerCallbackProvider : public JointStateListener
{
public:
  using JointStateListener::callbackJointState;
};

PYBIND11_MODULE(CppJointStateListener, m) {
  py::class_<JointStateListener>(m, "JointStateListener")
    .def(py::init<py::object, py::object, py::object, const KDL::Tree &, const urdf::Model &>())
    .def("callbackJointState", &JointStateListenerCallbackProvider::callbackJointState);
}
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$JOINTSTATELISTENER_CPP" -in-place

cd ../include/pdw_robot_state_publisher

COMBY_M=$(cat <<MATCH
#include:[1]#include <urdf/model.h>
#include:[2]#include <kdl/tree.hpp>
MATCH
)

COMBY_R=$(cat <<REWRITE
#include <urdf/model.h>
#include <kdl/tree.hpp>
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_H" -in-place

COMBY_M=$(cat <<MATCH
namespace :[[1]] {:[2]class RobotStatePublisher {:[3]RobotStatePublisher(:[4]);:[5]};:[6]}
MATCH
)

COMBY_R=$(cat <<REWRITE
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace :[1] {:[2]class RobotStatePublisher
{:[3]RobotStatePublisher(py::object tf_pub, py::object tf_static_pub, py::object ros, :[4]);:[5]
private:
  py::object tf_pub;
  py::object tf_static_pub;
  py::object ros;
};:[6]}
REWRITE
)
#namespace robot_state_publisher {:[1]}

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_H" -in-place

COMBY_M=$(cat <<MATCH
 ros::Time& :[[1]]
MATCH
)

COMBY_R=$(cat <<REWRITE
 py::object& :[1]
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_H" -in-place

COMBY_M=$(cat <<MATCH
tf2_ros::TransformBroadcaster :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_H" -in-place

COMBY_M=$(cat <<MATCH
tf2_ros::StaticTransformBroadcaster :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_H" -in-place

COMBY_M=$(cat <<MATCH
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_H" -in-place

cd ../../src

COMBY_M=$(cat <<MATCH
#include <:[1]>
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
using namespace ros;
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
namespace :[[1]]:[ 2]{:[3]}
MATCH
)

COMBY_R=$(cat <<REWRITE
namespace :[1] {

py::object kdlToTransform(py::object ros, const KDL::Frame &k)
{
  py::object t = ros.attr("messages").attr("geometry_msgs").attr("TransformStamped")();

  t.attr("transform").attr("translation").attr("x") = k.p.x();
  t.attr("transform").attr("translation").attr("y") = k.p.y();
  t.attr("transform").attr("translation").attr("z") = k.p.z();

  double q_x, q_y, q_z, q_w;
  k.M.GetQuaternion(q_x, q_y, q_z, q_w);

  t.attr("transform").attr("rotation").attr("x") = q_x;
  t.attr("transform").attr("rotation").attr("y") = q_y;
  t.attr("transform").attr("rotation").attr("z") = q_z;
  t.attr("transform").attr("rotation").attr("w") = q_w;

  return t;
}

std::string tf_strip_leading_slash(const std::string& frame_name)
{
  if (frame_name.size() > 0)
    if (frame_name[0] == '/')
    {
      std::string shorter = frame_name;
      shorter.erase(0,1);
      return shorter;
    }
  
  return frame_name;
}

std::string tf_resolve(const std::string& prefix, const std::string& frame_name)
{
  //  printf ("resolveping prefix:%s with frame_name:%s\n", prefix.c_str(), frame_name.c_str());
  if (frame_name.size() > 0)
    if (frame_name[0] == '/')
    {
      return tf_strip_leading_slash(frame_name);
    }
  if (prefix.size() > 0)
  {
    if (prefix[0] == '/')
    {
      std::string composite = tf_strip_leading_slash(prefix);
      composite.append("/");
      composite.append(frame_name);
      return composite;
    }
    else
    {
      std::string composite;
      composite.append(prefix);
      composite.append("/");
      composite.append(frame_name);
      return composite;
    }

  }
  else
  {
    std::string composite;
    composite.append(frame_name);
    return composite;
  }
}:[3]}
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
RobotStatePublisher::RobotStatePublisher(:[1]) ::[2] {:[3]}
MATCH
)

COMBY_R=$(cat <<REWRITE
RobotStatePublisher::RobotStatePublisher(py::object tf_pub, py::object tf_static_pub, py::object ros, :[1])
  ::[2], tf_pub(tf_pub), tf_static_pub(tf_static_pub), ros(ros)
{:[3]}
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
ROS_INFO(":[1]":[2])
MATCH
)

COMBY_R=$(cat <<REWRITE
fprintf(stderr, ":[1]""\n":[2])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
ROS_DEBUG(":[1]":[2])
MATCH
)

COMBY_R=$(cat <<REWRITE
fprintf(stderr, ":[1]""\n":[2])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
 Time& :[[1]]
MATCH
)

COMBY_R=$(cat <<REWRITE
 py::object& :[1]
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
std::vector<geometry_msgs::TransformStamped> :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1] = ros.attr("tf").attr("createTfList")();
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

# This wouldn't work in all cases.
COMBY_M=$(cat <<MATCH
ROS_WARN_THROTTLE(:[1], ":[2]",:[3])
MATCH
)

COMBY_R=$(cat <<REWRITE
fprintf(stderr, ":[2]""\n",:[3])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
geometry_msgs::TransformStamped :[[1]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1] =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
tf2::kdlToTransform(:[1])
MATCH
)

COMBY_R=$(cat <<REWRITE
kdlToTransform(ros, :[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
tf::resolve(:[1])
MATCH
)

COMBY_R=$(cat <<REWRITE
tf_resolve(:[1])
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
tf_transform.:[[1]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
tf_transform.attr(":[1]") =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
tf_transform.:[[1]].:[[2]] =
MATCH
)

COMBY_R=$(cat <<REWRITE
tf_transform.attr(":[1]").attr(":[2]") =
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
tf_transforms.push_back(:[1]);
MATCH
)

COMBY_R=$(cat <<REWRITE
tf_transforms.attr("add")(:[1]);
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

# SIN: We need to generate calls to different functions in the port object
#      based on the type of the argument passed to the sendTransform()
#      function.
COMBY_M=$(cat <<MATCH
:[ 1]tf_broadcaster_.sendTransform(:[2]);
MATCH
)

COMBY_R=$(cat <<REWRITE
:[1]tf_pub.attr("getTfListConsumed")(:[2]);
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
:[ 1]static_tf_broadcaster_.sendTransform(:[2]);
MATCH
)

COMBY_R=$(cat <<REWRITE
:[1]tf_static_pub.attr("getTfListConsumed")(:[2]);
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
geometry_msgs::TransformStamped :[[1]];
MATCH
)

COMBY_R=$(cat <<REWRITE
py::object :[1] = ros.attr("messages").attr("geometry_msgs").attr("TransformStamped")();
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
ros::Time::now()
MATCH
)

COMBY_R=$(cat <<REWRITE
ros.attr("time").attr("now")()
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
tf_transform.:[[1]].:[[2]] += ros::Duration(0.5);
MATCH
)

# For this rewrite, the following things needs to be figured out:
# 1. The ROS2 Duration class does not have a constructor that takes a double argument.
#    The constructor with a common signature that can be used for 0.5 seconds is the
#    one that takes two arguments - an integer value for seconds and another integer
#    value for nanoseconds.
# 2. The ROS2 Time class does not overload the += operator.
# 
# In the light of these facts, we rewrite this as given below.
COMBY_R=$(cat <<REWRITE
tf_transform.attr(":[1]").attr(":[2]") = ((py::object) tf_transform.attr(":[1]").attr(":[2]"))
                                                  + ros.attr("createDurationFromSecondsAndNanoseconds")(0, 500000000);
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place

COMBY_M=$(cat <<MATCH
MATCH
)

COMBY_R=$(cat <<REWRITE
REWRITE
)

comby "$COMBY_M" "$COMBY_R" "$ROBOTSTATEPUBLISHER_CPP" -in-place
