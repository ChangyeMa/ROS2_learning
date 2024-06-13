#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class NumberCounterNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("number_counter") # MODIFY NAME
        self.number_count_ = 0
        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_number, 10)
        self.get_logger().info("Number count subscriber has been started")

        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.get_logger().info("Number count publisher has been started")

        self.service_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter)
        self.get_logger().info("Reset counter service has been started")

    def callback_number(self, msg):
        self.number_count_ += msg.data
        msg_new=Int64()
        msg_new.data=self.number_count_
        self.publisher_.publish(msg_new)

    def callback_reset_counter(self, request, response):
        if request.data:
            self.number_count_ = 0
            response.success = True
            response.message = "Counter has been reset"
        else:
            response.success = False
            response.message = "Counter has not been reset"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = NumberCounterNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()