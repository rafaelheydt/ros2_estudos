#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class ResetCountClientNode(Node):
    def __init__(self):
        super().__init__("reset_count_client_node") 
        self.client_ = self.create_client(SetBool, "reset_counter")

    def call_reset_counter(self):
        while not self.client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = SetBool.Request()
        request.data = True

        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_reset_counter)

    def callback_reset_counter(self, future):
        response = future.result()
        self.get_logger().info(f"Got response: {str(response.success)}, {str(response.message)}" )
        


def main(args=None):
    rclpy.init(args=args)
    node = ResetCountClientNode()
    node.call_reset_counter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
