#!/usr/bin/env python3
"""
gui_node.py  —  STARTER STUB. This is YOUR work to implement.

Goal: a PyQt5 + PyQtGraph node that is a live client of the controller.

Suggested behaviour (adapt as you like, document changes):
    - Subscribes to /joint_states and renders the arm live (links + joints).
    - Plots joint angles (and, if you do PID tracking, tracking error) over
      time with PyQtGraph.
    - Lets the operator enter pick and place targets and send them to the
      controller (service call or topic publish).
    - Shows telemetry: end-effector position, current mode, status.

THE KEY CHALLENGE: the ROS 2 executor and the Qt event loop must run together
without either one blocking the other. Spinning ROS in a background thread or
driving rclpy.spin_once from a QTimer are both acceptable — your handling of
this is a graded signal (multithreading / timer synchronisation).

You may reuse the look and feel of a standard PyQtGraph arm plot; the point of
this task is the ROS 2 integration, not pixel-perfect styling.
"""

import sys

import rclpy
from rclpy.node import Node

from PyQt5 import QtWidgets  # noqa: F401  (import here so missing deps fail loudly)
import pyqtgraph as pg  # noqa: F401

from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]


class GuiNode(Node):
    def __init__(self):
        super().__init__("gui_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        # TODO: create subscription to /joint_states, service client / publisher
        #       for targets, and hand data to the Qt window.
        self.get_logger().info("gui_node started (stub — implement me).")


def main(args=None):
    rclpy.init(args=args)
    # TODO: build the QApplication + main window, wire it to GuiNode, and run
    #       the Qt loop and the ROS 2 executor together.
    node = GuiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
