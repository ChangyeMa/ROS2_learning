from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld=LaunchDescription()
    robot_names=["giskard","bb8","daneel","jander","c3po"]
    robot_news_station_nodes=[]

    for name in robot_names:
        robot_news_station_node=Node(
            package="my_py_pkg",
            executable="robot_news_station",
            name="robot_news_station_"+name,
            parameters=[{"robot_name":name}]
        )
        robot_news_station_nodes.append(robot_news_station_node)
    
    smartphone=Node(
        package="my_py_pkg",
        executable="smartphone",
        name="smartphone",
    )

    for node in robot_news_station_nodes:
        ld.add_action(node)

    ld.add_action(smartphone)
    return ld