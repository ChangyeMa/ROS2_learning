#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("py_test")
        # the node name can be changed dynamcially when lanunching the node:
        # for example: "ros2 run my_py_pkg py_node --ros-args -r __node:=new_node_name"
        self.counter_=0
        self.get_logger().info("Hello ROS2")
        self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello"+ str(self.counter_))

def main(args=None):
    rclpy.init(args=args)   # needed to initialize the ROS2 client library for every ROS2 node

    # Regualr way to create a node
    # node = Node("py_test")
    # node.get_logger().info("Hello ROS2") # print message to the console

    # obejct-oriented way to create a node
    node = MyNode()

    rclpy.spin(node)         # needed to keep the node alive
    rclpy.shutdown()        # needed to shutdown the ROS2 client library for every ROS2 node

if __name__ == '__main__':
    main()


# Some Template:
# #OOP Python Code Template for Nodes

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
    
    
# class MyCustomNode(Node): # MODIFY NAME
#     def __init__(self):
#         super().__init__("node_name") # MODIFY NAME
    
    
# def main(args=None):
#     rclpy.init(args=args)
#     node = MyCustomNode() # MODIFY NAME
#     rclpy.spin(node)
#     rclpy.shutdown()
    
    
# if __name__ == "__main__":
#     main()


# #OOP C++ Code Template for Nodes

# #include "rclcpp/rclcpp.hpp"
    
# class MyCustomNode : public rclcpp::Node // MODIFY NAME
# {
# public:
#     MyCustomNode() : Node("node_name") // MODIFY NAME
#     {
#     }
    
# private:
# };
    
# int main(int argc, char **argv)
# {
#     rclcpp::init(argc, argv);
#     auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
#     rclcpp::spin(node);
#     rclcpp::shutdown();
#     return 0;
# }