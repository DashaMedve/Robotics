import sys
import time

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleMover(Node):

    def __init__(self, x, y, theta, e=1e-6):
        super().__init__("turtle_mover")
        
        self.desired_x = x
        self.desired_y = y
        self.desired_theta = theta
        
        self.current_x = -1
        self.current_y = -1
        self.current_theta = -1
        
        self.e = e
        self.x_ready = False
        self.y_ready = False
        self.theta_ready = False
        
        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        
        self.subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.callback,
            10
        )
        
    def callback(self, data):
        self.current_x = data.x
        self.current_y = data.y
        self.current_theta = data.theta
        
        self.get_logger().info("Current results:")
        self.get_logger().info(f"Desired x = {self.desired_x}\tCurrent x = {self.current_x}\tDifference = {abs(self.desired_x - self.current_x)}")
        self.get_logger().info(f"Desired y = {self.desired_y}\tCurrent y = {self.current_y}\tDifference = {abs(self.desired_y - self.current_y)}")
        self.get_logger().info(f"Desired theta = {self.desired_theta}\tCurrent theta = {self.current_theta}\tDifference = {abs(self.desired_theta - self.current_theta)}")
        self.get_logger().info("----------------------------------------------------------------------------------------")
        
        if self.current_x != -1 and self.current_y != -1 and self.current_theta != -1:
        
            message = Twist()
            
            if not self.x_ready or not self.y_ready:
                theta_difference = -self.current_theta
                self.get_logger().info(f"self.current_theta = {self.current_theta}")
                self.get_logger().info(f"theta_difference = {theta_difference}")
                self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                message.angular.z = theta_difference
                self.publisher.publish(message)
                time.sleep(1.5)
            
                if abs(theta_difference) < self.e:
                    message.angular.z = 0.0
                    x_difference = self.desired_x - self.current_x
                    y_difference = self.desired_y - self.current_y
                    self.get_logger().info(f"self.desired_x = {self.desired_x}")
                    self.get_logger().info(f"self.current_x = {self.current_x}")
                    self.get_logger().info(f"x_difference = {x_difference}")
                    self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                    self.get_logger().info(f"self.desired_y = {self.desired_y}")
                    self.get_logger().info(f"self.current_y = {self.current_y}")
                    self.get_logger().info(f"y_difference = {y_difference}")
                    self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                    if abs(x_difference) < self.e and abs(y_difference) < self.e:
                        message.linear.x = 0.0
                        message.linear.y = 0.0
                        self.x_ready = True
                        self.y_ready = True
                    message.linear.x = x_difference
                    message.linear.y = y_difference
                    self.publisher.publish(message)
                    time.sleep(1.5)
            
            elif not self.theta_ready:
                theta_difference = self.desired_theta - self.current_theta
                self.get_logger().info(f"self.desired_theta = {self.desired_theta}")
                self.get_logger().info(f"self.current_theta = {self.current_theta}")
                self.get_logger().info(f"theta_difference = {theta_difference}")
                self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                if abs(theta_difference) < self.e:
                    message.angular.z = 0.0
                    self.theta_ready = True
                message.angular.z = theta_difference
                self.publisher.publish(message)
                time.sleep(1.5)
            
            if self.x_ready and self.y_ready and self.theta_ready:
                raise SystemExit
    

def main(args=None):
    x, y, theta = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    rclpy.init(args=args)
    turtle_mover = TurtleMover(x, y, theta)
    try:
        rclpy.spin(turtle_mover)
    except SystemExit:
        turtle_mover.get_logger().info("***************************")
        turtle_mover.get_logger().info("****** Work is done! ******")
        turtle_mover.get_logger().info("***************************")
    turtle_mover.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
