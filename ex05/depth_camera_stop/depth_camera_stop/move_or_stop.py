import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class MovementPublisher(Node):

	def __init__(self):
		super().__init__('movement_publisher')
		self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
		timer_period = 1
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.subscription = self.create_subscription(
			Image, 
			'/depth/image', 
			self.listener_callback, 
			10)
		self.bridge = CvBridge()
		self.go = True

	def timer_callback(self):
		msg = Twist()
		if self.go:
			msg.linear.x = 0.5
		else:
			msg.linear.x = 0.0
		self.publisher.publish(msg)

	def listener_callback(self, msg):
		try:
			depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
		except CvBridgeError as e:
			print(f"CvBridgeError: {e}")
			return
		
		depth_array = np.array(depth_image, dtype=np.float32)
		h, w = depth_array.shape
		
		if depth_array[(h // 2 - 10):(h // 2 + 10), (w // 2 - 10):(w // 2 + 10)].sum() < 0.5 * 400:
			self.go = False
		else:
			self.go = True

def main(args=None):
	rclpy.init(args=args)
	movement_publisher = MovementPublisher()
	rclpy.spin(movement_publisher)
	movement_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

