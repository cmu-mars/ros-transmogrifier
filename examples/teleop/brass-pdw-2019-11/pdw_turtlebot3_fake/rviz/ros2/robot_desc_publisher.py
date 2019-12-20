import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from std_msgs.msg import String

import sys

class RobotDescriptionPublisher(Node):
	def __init__(self, robot_description):
		super().__init__('robot_desc_publisher')

		qos_profile = QoSProfile(depth=1)
		qos_profile.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
		self.publisher_ = self.create_publisher(String, 'robot_description', qos_profile)
		self.robot_description = robot_description

		self.timer_callback()

		#timer_period = 1 # seconds
		#self.timer = self.create_timer(timer_period, self.timer_callback)

	def timer_callback(self):
		msg = String()
		msg.data = self.robot_description
		self.publisher_.publish(msg)

def main(args=None):
	try:
		urdf_file_path = sys.argv[1]

		with open(urdf_file_path, 'r') as urdf_file:
			robot_description = urdf_file.read()

		rclpy.init(args=args)

		robot_desc_publisher = RobotDescriptionPublisher(robot_description)

		rclpy.spin(robot_desc_publisher)
	except IndexError:
		print("Usage: python3 robot_desc_publisher.py <path to URDF file>")

if __name__ == '__main__':
	main()
