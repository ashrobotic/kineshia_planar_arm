"""
bringup.launch.py  —  STARTER STUB.

Launch the controller node and the GUI node together. Fill in the nodes once
you have implemented them, and expose any parameters you add (publish rate,
control mode, trajectory duration, etc.) here.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="planar_arm_control",
            executable="controller_node",
            name="controller_node",
            output="screen",
            # parameters=[{"publish_rate_hz": 50.0, "control_mode": "position"}],
        ),
        Node(
            package="planar_arm_control",
            executable="gui_node",
            name="gui_node",
            output="screen",
        ),
    ])
