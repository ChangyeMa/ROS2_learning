#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_no_oop")

    client = node.create_client(AddTwoInts, "add_two_ints")
    while not client.wait_for_service(1.0):  # waiting time of 1 second
        node.get_logger().warn("Waiting for the server...")

    request = AddTwoInts.Request()
    request.a = 3
    request.b = 4

    # call the service asynchronously to avoid error 
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(f"{request.a} + {request.b} = {response.sum}")

    except Exception as e:
        node.get_logger().error(f"Service call failed: {str(e)}")

    rclpy.shutdown()

if __name__ == "__main__":
    main()