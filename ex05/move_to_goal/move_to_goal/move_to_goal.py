import sys
import time
import math

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class TurtleMover(Node):

    def __init__(self, x, y, theta, e=0.1):
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
            
            x_difference = self.desired_x - self.current_x
            y_difference = self.desired_y - self.current_y
            
            if abs(x_difference) < self.e:
                self.x_ready = True
            if abs(y_difference) < self.e:
                self.y_ready = True
            
            if not self.x_ready or not self.y_ready:
                temporary_desired_theta = math.atan(y_difference / (x_difference + 1e-10))
                if x_difference < 0:
                    if y_difference < 0:
                        temporary_desired_theta -= math.pi
                    else:
                        temporary_desired_theta += math.pi
                theta_difference = temporary_desired_theta - self.current_theta
                
                self.get_logger().info(f"temporary_desired_theta = {temporary_desired_theta}")
                self.get_logger().info(f"self.current_theta = {self.current_theta}")
                self.get_logger().info(f"theta_difference = {theta_difference}")
                self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                self.get_logger().info(f"self.desired_x = {self.desired_x}")
                self.get_logger().info(f"self.current_x = {self.current_x}")
                self.get_logger().info(f"x_difference = {x_difference}")
                self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                self.get_logger().info(f"self.desired_y = {self.desired_y}")
                self.get_logger().info(f"self.current_y = {self.current_y}")
                self.get_logger().info(f"y_difference = {y_difference}")
                self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                
                if abs(theta_difference) < self.e / 10:
                    s = math.sqrt(x_difference**2 + y_difference**2)
                    message.linear.x = s
                    self.publisher.publish(message)
                    time.sleep(1.5)
                else:
                    message.linear.x = abs(theta_difference) * 0.5
                    message.angular.z = theta_difference
                    self.publisher.publish(message)
                    time.sleep(1.5)
            
            elif not self.theta_ready:
                theta_difference = self.desired_theta - self.current_theta
                self.get_logger().info(f"self.desired_theta = {self.desired_theta}")
                self.get_logger().info(f"self.current_theta = {self.current_theta}")
                self.get_logger().info(f"theta_difference = {theta_difference}")
                self.get_logger().info("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                if abs(theta_difference) < self.e:
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
