from launch import LaunchDescription
from launch_ros.actions import Node

device_name = "agv0"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dwm1001",
            executable="dwm1001",
            namespace=device_name,
            name="dwm1001",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"port_name": "ttyS0"}
            ],
            remappings=[
                ("dwm1001/output/error", "output/error"),
                ("dwm1001/output/configuration", "output/configuration"),
                ("dwm1001/output/status", "output/status"),
                ("dwm1001/output/position", "output/position"),
            ]
        )
    ])