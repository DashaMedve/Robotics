import sys

from new_message_type_package.srv import FullNameSumService
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(FullNameSumService, 'SummFullName')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FullNameSumService.Request()

    def send_request(self, last_name, first_name, name):
        self.req.last_name = last_name
        self.req.first_name = first_name
        self.req.name = name
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(sys.argv[1], sys.argv[2], sys.argv[3])
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(
        'Result of SummFullName: for %s + %s + %s = %s (last_name + first_name + name = full_name)' %
        (sys.argv[1], sys.argv[2], sys.argv[3], response.full_name))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

