#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedStates
from my_robot_interfaces.srv import SetLed
from example_interfaces.msg import Int64


class BatteryNode(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("battery") # MODIFY NAME
        self.counter_ = 0
        self.battery_level = 0        
        self.get_logger().info("Battery node has been started")
        self.publisher_ = self.create_publisher(Int64, "battery_level", 10)
        self.create_timer(1, self.timer_callback)
        self.call_led_server(self.battery_level)

    def timer_callback(self):
        self.counter_ += 1
        if self.counter_ % 5 == 0:
            self.battery_level = 1
        else:
            self.battery_level = 0
        msg = Int64()
        msg.data = self.battery_level
        self.publisher_.publish(msg)
        self.get_logger().info(f"Battery level: {self.battery_level}")

    def call_led_server(self,battery_level):
        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the server...")
        request = SetLed.Request()
        request.soc = battery_level
        future = client.call_async(request)
        future.add_done_callback(self.callback_call_led_server)
    
    def callback_call_led_server(self,future):
        try:
            response = future.result()
            self.get_logger().info(f"LED state: {response.states}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()