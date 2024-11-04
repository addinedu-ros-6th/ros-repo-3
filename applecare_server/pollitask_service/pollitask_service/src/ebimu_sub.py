#This is a code for wired sensors (ebimu-9dof)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def data_parser(msg_data):
	words = msg_data.split(",")    # Fields split

	if(-1 < words[0].find('*')) :
		words[0]=words[0].replace('*','')
		return list(map(float, words)) # float type


class EbimuSubscriber(Node):

	def __init__(self):
		super().__init__('ebimu_subscriber')
		qos_profile = QoSProfile(depth=10)
		qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
		#qos_profile.reliability = ReliabilityPolicy.KeepLast
		self.subscription = self.create_subscription(String, 'ebimu_data', self.callback, qos_profile)
		self.subscription   # prevent unuse variable warning

	def callback(self, msg):
		imu_data = data_parser(msg.data)
		print(imu_data)



def main(args=None):
	rclpy.init(args=args)

	print("Starting ebimu_subscriber..")

	node = EbimuSubscriber()

	try:
		rclpy.spin(node)

	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
