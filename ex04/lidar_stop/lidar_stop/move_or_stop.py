import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MovementPublisher(Node):

	def __init__(self):
		super().__init__('movement_publisher')
		self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
		timer_period = 1
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.subscription = self.create_subscription(LaserScan, '/robot/scan', self.listener_callback, 10)
		self.go = True

	def timer_callback(self):
		msg = Twist()
		if self.go:
			msg.linear.x = 0.5
		else:
			msg.linear.x = 0.0
		self.publisher.publish(msg)

	def listener_callback(self, msg):
		ranges = msg.ranges
		n = len(ranges)
		c = 0
		for i in range(n // 2 - 15, n // 2 + 15):
			if ranges[i] < 0.5:
				self.go = False
				break
			else:
				c += 1
		if c == 30:
			self.go = True

def main(args=None):
	rclpy.init(args=args)
	movement_publisher = MovementPublisher()
	rclpy.spin(movement_publisher)
	movement_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

