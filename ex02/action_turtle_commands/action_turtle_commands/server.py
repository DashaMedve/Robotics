import time
import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from turtle_action_message_interface.action import MessageTurtleCommands
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class PoseSubscriber(Node):
    
    def __init__(self):
        super().__init__("pose_subscriber")
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.listener_callback,
            10
        )
        self.last_x = 0
        self.last_y = 0
        self.last_theta = 0
        self.current_distance = 0
        
    def listener_callback(self, data):
        x = data.x
        y = data.y
        self.last_theta = 2 * math.pi + data.theta if data.theta < 0 else data.theta
        self.current_distance += math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
        self.last_x = x
        self.last_y = y


class TurtleActionServer(Node):

    def __init__(self, pose_subscriber):
        super().__init__('action_server')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'MessageTurtleCommands',
            self.execute_callback)
        self._cmd_vel_publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        self._pose_subscriber = pose_subscriber
        

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = MessageTurtleCommands.Feedback()
        self._pose_subscriber.current_distance = 0
        
        result = MessageTurtleCommands.Result()
        result.result = False
        
        message = Twist()
        
        if goal_handle.request.command == "forward":
            self.get_logger().info('------ GOING FORWARD ------')
            message.linear.x = float(goal_handle.request.s)
            self._cmd_vel_publisher.publish(message)
            while int(self._pose_subscriber.current_distance) != goal_handle.request.s:
                self.get_logger().info(f'Current distance = {self._pose_subscriber.current_distance}')
                self.get_logger().info(f'Desired distance = {goal_handle.request.s}')
                self.get_logger().info('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
                feedback_msg.odom = int(self._pose_subscriber.current_distance)
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.05)
        elif goal_handle.request.command == "turn_left" or goal_handle.request.command == "turn_right":
            if goal_handle.request.command == "turn_left":
                self.get_logger().info('------ TURNING LEFT ------')
                message.angular.z = goal_handle.request.angle * math.pi / 180
            else:
                self.get_logger().info('------ TURNING RIGHT ------')
                message.angular.z = -goal_handle.request.angle * math.pi / 180
            initial_theta = self._pose_subscriber.last_theta
            k = (initial_theta + message.angular.z) // (2 * math.pi)
            desired_theta = (initial_theta + message.angular.z) - 2 * math.pi * k
            difference = desired_theta
            self._cmd_vel_publisher.publish(message)
            while True:
                self.get_logger().info(f'Current theta = {self._pose_subscriber.last_theta}')
                self.get_logger().info(f'Desired theta = {desired_theta}')
                self.get_logger().info('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(0.05)
                if math.isclose(desired_theta - self._pose_subscriber.last_theta, difference):
                    if difference > 6:
                        if desired_theta > 6:
                            message.angular.z = -math.pi / 180
                        else:
                            message.angular.z = math.pi / 180
                    else:
                        message.angular.z = difference / 2
                    self._cmd_vel_publisher.publish(message)
                difference = desired_theta - self._pose_subscriber.last_theta
                if abs(difference) <= math.pi / 180:
                    break
        else:
            return result
        
        goal_handle.succeed()
        result.result = True
        return result
        

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor(num_threads=2)

    pose_subscriber = PoseSubscriber()
    action_server = TurtleActionServer(pose_subscriber)

    executor.add_node(pose_subscriber)
    executor.add_node(action_server)
    executor.spin()


if __name__ == '__main__':
    main()

