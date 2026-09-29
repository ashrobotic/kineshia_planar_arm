#!/usr/bin/env python3
"""
controller_node.py  —  STARTER STUB. This is YOUR work to implement.

Goal: a ROS 2 node that owns the arm state, accepts a target, plans a
time-parameterized joint trajectory, and streams joint states as it executes.

Suggested interface (you may adapt, but document any changes in your README):
    - Publishes:   /joint_states   (sensor_msgs/JointState)   at a fixed rate
    - Service:     /move_to_target (your choice of srv; e.g. a Point target)
      OR Topic:    /target_pose    (geometry_msgs/PointStamped)
    - Parameters:  publish_rate_hz, control_mode, trajectory_duration, ...

Core requirements (see the task brief):
    1. Run IK on the incoming target (use PlanarArm.inverse_kinematics).
    2. Generate a smooth joint-space trajectory from the current q to the goal
       q (trapezoidal or quintic — your choice; explain it).
    3. Step along the trajectory in a timer callback and publish JointState.
    4. Respect joint limits and the ground constraint.
    5. Design the command path so a hardware backend (e.g. Dynamixel) could be
       swapped in later without rewriting the planner.

Stretch (optional, rewarded): velocity / current control modes, a PID
trajectory-tracking loop with an error signal, a ROS 2 action for the full
pick-and-place with feedback/cancel.
"""

import rclpy
from rclpy.node import Node

# The provided kinematics library — do not modify it.
from planar_arm_control.planar_arm import PlanarArm

LINK_LENGTHS = [3.0, 2.0, 1.5]


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.arm = PlanarArm(LINK_LENGTHS)
        # TODO: declare parameters, create publishers/services, timers, state.
        self.get_logger().info("controller_node started (stub — implement me).")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
