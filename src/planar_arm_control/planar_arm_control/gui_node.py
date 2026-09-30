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
import time
import math
from collections import deque

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

from planar_arm_control.planar_arm import PlanarArm
from planar_arm_control.srv import MoveToTarget


# ============================================================================
# Configuration
# ============================================================================

LINK_LENGTHS = [3.0, 2.0, 1.5]

JOINT_NAMES = [
    "joint1",
    "joint2",
    "joint3",
]

JOINT_STATE_TOPIC = "/joint_states"
MOVE_SERVICE = "/move_to_target"

GUI_UPDATE_HZ = 30.0
PLOT_HISTORY_SECONDS = 20.0

PICK_TARGET = (4.0, 2.0)
PLACE_TARGET = (3.0, 3.0)

# Visualization limits
X_MIN = -7.0
X_MAX = 7.0
Y_MIN = -1.0
Y_MAX = 7.0


# ============================================================================
# ROS2 Node
# ============================================================================

class GuiNode(Node):
    """ROS2 interface used by the Qt GUI."""

    def __init__(self):

        super().__init__("gui_node")

        self.arm = PlanarArm(LINK_LENGTHS)

        # Latest robot state
        self.latest_positions = [0.0, 0.0, 0.0]
        self.latest_velocities = [0.0, 0.0, 0.0]

        self.last_joint_state_time = time.monotonic()

        # ------------------------------------------------------------------
        # Joint state subscriber
        # ------------------------------------------------------------------

        self.joint_subscription = self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self._joint_state_callback,
            10,
        )

        # ------------------------------------------------------------------
        # Move service client
        # ------------------------------------------------------------------

        self.move_client = self.create_client(
            MoveToTarget,
            MOVE_SERVICE,
        )

        self.get_logger().info(
            "gui_node started."
        )

    # ----------------------------------------------------------------------
    # Joint state callback
    # ----------------------------------------------------------------------

    def _joint_state_callback(self, msg: JointState):
        """Store the latest joint positions and velocities."""

        positions = [0.0, 0.0, 0.0]
        velocities = [0.0, 0.0, 0.0]

        for i, joint_name in enumerate(JOINT_NAMES):

            if joint_name not in msg.name:
                continue

            index = msg.name.index(joint_name)

            if index < len(msg.position):
                positions[i] = float(
                    msg.position[index]
                )

            if index < len(msg.velocity):
                velocities[i] = float(
                    msg.velocity[index]
                )

        self.latest_positions = positions
        self.latest_velocities = velocities

        self.last_joint_state_time = time.monotonic()

    # ----------------------------------------------------------------------
    # Send Cartesian target
    # ----------------------------------------------------------------------

    def send_target(self, x: float, y: float):
        """Send a Cartesian target to the controller."""

        if not self.move_client.service_is_ready():
            return None

        request = MoveToTarget.Request()

        request.x = float(x)
        request.y = float(y)

        return self.move_client.call_async(
            request
        )


# ============================================================================
# Main Window
# ============================================================================

class MainWindow(QtWidgets.QMainWindow):
    """Main PyQt5 GUI."""

    def __init__(self, node: GuiNode):

        super().__init__()

        self.node = node

        self.setWindowTitle(
            "Kineshia Robotics - 3-DoF Planar Manipulator"
        )

        self.resize(
            1250,
            800
        )

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------

        self.pending_future = None

        self.sequence_active = False
        self.sequence_step = 0
        self.sequence_wait_start = None

        self.start_time = time.monotonic()

        # ------------------------------------------------------------------
        # Plot history
        # ------------------------------------------------------------------

        max_samples = int(
            PLOT_HISTORY_SECONDS * GUI_UPDATE_HZ
        )

        self.time_history = deque(
            maxlen=max_samples
        )

        self.q1_history = deque(
            maxlen=max_samples
        )

        self.q2_history = deque(
            maxlen=max_samples
        )

        self.q3_history = deque(
            maxlen=max_samples
        )

        # ------------------------------------------------------------------
        # Build UI
        # ------------------------------------------------------------------

        self._build_ui()

        # ------------------------------------------------------------------
        # Qt timer
        #
        # Qt remains the owner of the event loop.
        # ROS callbacks are processed without blocking.
        # ------------------------------------------------------------------

        self.timer = QtCore.QTimer(self)

        self.timer.timeout.connect(
            self._update
        )

        self.timer.start(
            int(1000.0 / GUI_UPDATE_HZ)
        )

    # ======================================================================
    # UI
    # ======================================================================

    def _build_ui(self):

        central = QtWidgets.QWidget()

        self.setCentralWidget(
            central
        )

        main_layout = QtWidgets.QHBoxLayout(
            central
        )

        # ==================================================================
        # LEFT SIDE
        # ==================================================================

        left_layout = QtWidgets.QVBoxLayout()

        # ------------------------------------------------------------------
        # Live Arm Visualization
        # ------------------------------------------------------------------

        arm_group = QtWidgets.QGroupBox(
            "Live Manipulator"
        )

        arm_layout = QtWidgets.QVBoxLayout()

        self.arm_plot = pg.PlotWidget()

        self.arm_plot.setBackground(
            "w"
        )

        self.arm_plot.setAspectLocked(
            True
        )

        self.arm_plot.setXRange(
            X_MIN,
            X_MAX,
            padding=0
        )

        self.arm_plot.setYRange(
            Y_MIN,
            Y_MAX,
            padding=0
        )

        self.arm_plot.setLabel(
            "bottom",
            "X",
            units="m"
        )

        self.arm_plot.setLabel(
            "left",
            "Y",
            units="m"
        )

        self.arm_plot.showGrid(
            x=True,
            y=True,
            alpha=0.25
        )

        # --------------------------------------------------------------
        # Ground
        # --------------------------------------------------------------

        self.ground_line = pg.PlotDataItem(
            x=[X_MIN, X_MAX],
            y=[0.0, 0.0],
            pen=pg.mkPen(
                color=(80, 80, 80),
                width=2
            )
        )

        self.arm_plot.addItem(
            self.ground_line
        )

        # --------------------------------------------------------------
        # Workspace boundary
        #
        # Maximum reach = 6.5 m.
        # This is only a visual reference.
        # --------------------------------------------------------------

        workspace_x = []
        workspace_y = []

        for i in range(181):

            theta = math.radians(i)

            workspace_x.append(
                6.5 * math.cos(theta)
            )

            workspace_y.append(
                6.5 * math.sin(theta)
            )

        self.workspace_line = pg.PlotDataItem(
            x=workspace_x,
            y=workspace_y,
            pen=pg.mkPen(
                color=(170, 170, 170),
                width=1,
                style=QtCore.Qt.DashLine
            )
        )

        self.arm_plot.addItem(
            self.workspace_line
        )

        # --------------------------------------------------------------
        # Arm links
        # --------------------------------------------------------------

        self.arm_line = pg.PlotDataItem(
            pen=pg.mkPen(
                color=(40, 90, 180),
                width=7
            )
        )

        self.arm_plot.addItem(
            self.arm_line
        )

        # --------------------------------------------------------------
        # Joint markers
        # --------------------------------------------------------------

        self.joint_points = pg.ScatterPlotItem(
            size=17,
            brush=pg.mkBrush(
                40,
                90,
                180
            ),
            pen=pg.mkPen(
                color=(20, 40, 80),
                width=2
            )
        )

        self.arm_plot.addItem(
            self.joint_points
        )

        # --------------------------------------------------------------
        # End effector
        # --------------------------------------------------------------

        self.ee_point = pg.ScatterPlotItem(
            size=23,
            brush=pg.mkBrush(
                220,
                70,
                50
            ),
            pen=pg.mkPen(
                color=(120, 30, 20),
                width=2
            )
        )

        self.arm_plot.addItem(
            self.ee_point
        )

        arm_layout.addWidget(
            self.arm_plot
        )

        arm_group.setLayout(
            arm_layout
        )

        left_layout.addWidget(
            arm_group,
            stretch=3
        )

        # ==================================================================
        # JOINT ANGLE GRAPH
        # ==================================================================

        graph_group = QtWidgets.QGroupBox(
            "Joint Angles vs Time"
        )

        graph_layout = QtWidgets.QVBoxLayout()

        self.joint_plot = pg.PlotWidget()

        self.joint_plot.setBackground(
            "w"
        )

        self.joint_plot.setLabel(
            "bottom",
            "Time",
            units="s"
        )

        self.joint_plot.setLabel(
            "left",
            "Joint Angle",
            units="deg"
        )

        self.joint_plot.showGrid(
            x=True,
            y=True,
            alpha=0.25
        )

        self.joint_plot.setYRange(
            -130,
            190
        )

        self.joint_plot.addLegend()

        # --------------------------------------------------------------
        # Curves
        # --------------------------------------------------------------

        self.q1_curve = self.joint_plot.plot(
            pen=pg.mkPen(
                color=(40, 90, 180),
                width=2
            ),
            name="Joint 1"
        )

        self.q2_curve = self.joint_plot.plot(
            pen=pg.mkPen(
                color=(220, 80, 50),
                width=2
            ),
            name="Joint 2"
        )

        self.q3_curve = self.joint_plot.plot(
            pen=pg.mkPen(
                color=(40, 150, 80),
                width=2
            ),
            name="Joint 3"
        )

        graph_layout.addWidget(
            self.joint_plot
        )

        # --------------------------------------------------------------
        # Clear history button
        # --------------------------------------------------------------

        self.clear_history_button = QtWidgets.QPushButton(
            "Clear Joint History"
        )

        self.clear_history_button.clicked.connect(
            self._clear_plot_history
        )

        graph_layout.addWidget(
            self.clear_history_button
        )

        graph_group.setLayout(
            graph_layout
        )

        left_layout.addWidget(
            graph_group,
            stretch=2
        )

        main_layout.addLayout(
            left_layout,
            stretch=3
        )

        # ==================================================================
        # RIGHT SIDE
        # ==================================================================

        right_layout = QtWidgets.QVBoxLayout()

        # ==================================================================
        # TELEMETRY
        # ==================================================================

        telemetry_group = QtWidgets.QGroupBox(
            "Live Telemetry"
        )

        telemetry_layout = QtWidgets.QFormLayout()

        self.ee_x_label = QtWidgets.QLabel(
            "0.000 m"
        )

        self.ee_y_label = QtWidgets.QLabel(
            "0.000 m"
        )

        self.q1_label = QtWidgets.QLabel(
            "0.00°"
        )

        self.q2_label = QtWidgets.QLabel(
            "0.00°"
        )

        self.q3_label = QtWidgets.QLabel(
            "0.00°"
        )

        self.mode_label = QtWidgets.QLabel(
            "Position"
        )

        self.status_label = QtWidgets.QLabel(
            "Starting..."
        )

        self.status_label.setWordWrap(
            True
        )

        self.status_label.setMinimumHeight(
            45
        )

        telemetry_layout.addRow(
            "EE X:",
            self.ee_x_label
        )

        telemetry_layout.addRow(
            "EE Y:",
            self.ee_y_label
        )

        telemetry_layout.addRow(
            "Joint 1:",
            self.q1_label
        )

        telemetry_layout.addRow(
            "Joint 2:",
            self.q2_label
        )

        telemetry_layout.addRow(
            "Joint 3:",
            self.q3_label
        )

        telemetry_layout.addRow(
            "Mode:",
            self.mode_label
        )

        telemetry_layout.addRow(
            "Status:",
            self.status_label
        )

        telemetry_group.setLayout(
            telemetry_layout
        )

        right_layout.addWidget(
            telemetry_group
        )

        # ==================================================================
        # CARTESIAN TARGET
        # ==================================================================

        target_group = QtWidgets.QGroupBox(
            "Cartesian Target"
        )

        target_layout = QtWidgets.QFormLayout()

        self.x_input = QtWidgets.QDoubleSpinBox()

        self.x_input.setRange(
            -7.0,
            7.0
        )

        self.x_input.setDecimals(
            3
        )

        self.x_input.setSingleStep(
            0.1
        )

        self.x_input.setValue(
            4.0
        )

        self.y_input = QtWidgets.QDoubleSpinBox()

        self.y_input.setRange(
            -1.0,
            7.0
        )

        self.y_input.setDecimals(
            3
        )

        self.y_input.setSingleStep(
            0.1
        )

        self.y_input.setValue(
            2.0
        )

        self.move_button = QtWidgets.QPushButton(
            "Move to Target"
        )

        self.move_button.clicked.connect(
            self._move_to_target
        )

        target_layout.addRow(
            "X (m):",
            self.x_input
        )

        target_layout.addRow(
            "Y (m):",
            self.y_input
        )

        target_layout.addRow(
            self.move_button
        )

        target_group.setLayout(
            target_layout
        )

        right_layout.addWidget(
            target_group
        )

        # ==================================================================
        # PICK / PLACE
        # ==================================================================

        sequence_group = QtWidgets.QGroupBox(
            "Pick / Place"
        )

        sequence_layout = QtWidgets.QVBoxLayout()

        sequence_layout.addWidget(
            QtWidgets.QLabel(
                "Pick target: (4.0, 2.0)"
            )
        )

        self.pick_button = QtWidgets.QPushButton(
            "Move to Pick"
        )

        self.pick_button.clicked.connect(
            self._move_to_pick
        )

        sequence_layout.addWidget(
            self.pick_button
        )

        sequence_layout.addWidget(
            QtWidgets.QLabel(
                "Place target: (3.0, 3.0)"
            )
        )

        self.place_button = QtWidgets.QPushButton(
            "Move to Place"
        )

        self.place_button.clicked.connect(
            self._move_to_place
        )

        sequence_layout.addWidget(
            self.place_button
        )

        self.pick_place_button = QtWidgets.QPushButton(
            "Run Pick → Move → Place"
        )

        self.pick_place_button.clicked.connect(
            self._start_pick_place
        )

        sequence_layout.addWidget(
            self.pick_place_button
        )

        sequence_group.setLayout(
            sequence_layout
        )

        right_layout.addWidget(
            sequence_group
        )

        # ==================================================================
        # TEST SCENARIOS
        # ==================================================================

        test_group = QtWidgets.QGroupBox(
            "Test Scenarios"
        )

        test_layout = QtWidgets.QVBoxLayout()

        # Pick
        pick_test = QtWidgets.QPushButton(
            "Test Pick (4.0, 2.0)"
        )

        pick_test.clicked.connect(
            lambda: self._send_coordinates(
                4.0,
                2.0
            )
        )

        test_layout.addWidget(
            pick_test
        )

        # Place
        place_test = QtWidgets.QPushButton(
            "Test Place (3.0, 3.0)"
        )

        place_test.clicked.connect(
            lambda: self._send_coordinates(
                3.0,
                3.0
            )
        )

        test_layout.addWidget(
            place_test
        )

        # Unreachable
        unreachable_test = QtWidgets.QPushButton(
            "Test Unreachable (7.0, 3.0)"
        )

        unreachable_test.clicked.connect(
            lambda: self._send_coordinates(
                7.0,
                3.0
            )
        )

        test_layout.addWidget(
            unreachable_test
        )

        test_group.setLayout(
            test_layout
        )

        right_layout.addWidget(
            test_group
        )

        # ==================================================================
        # ROS CONNECTION STATUS
        # ==================================================================

        connection_group = QtWidgets.QGroupBox(
            "ROS2 System"
        )

        connection_layout = QtWidgets.QFormLayout()

        self.ros_status_label = QtWidgets.QLabel(
            "Checking..."
        )

        self.joint_state_status_label = QtWidgets.QLabel(
            "Waiting..."
        )

        connection_layout.addRow(
            "Move service:",
            self.ros_status_label
        )

        connection_layout.addRow(
            "Joint states:",
            self.joint_state_status_label
        )

        connection_group.setLayout(
            connection_layout
        )

        right_layout.addWidget(
            connection_group
        )

        # Push remaining content upward
        right_layout.addStretch()

        main_layout.addLayout(
            right_layout,
            stretch=1
        )

    # ======================================================================
    # Main update loop
    # ======================================================================

    def _update(self):

        # ------------------------------------------------------------------
        # ROS callbacks
        # ------------------------------------------------------------------

        try:

            rclpy.spin_once(
                self.node,
                timeout_sec=0.0
            )

        except Exception as exc:

            self.status_label.setText(
                f"ROS error: {exc}"
            )

            return

        # ------------------------------------------------------------------
        # Service response
        # ------------------------------------------------------------------

        self._check_service_response()

        # ------------------------------------------------------------------
        # Visualization
        # ------------------------------------------------------------------

        self._update_arm_visualization()

        # ------------------------------------------------------------------
        # Telemetry
        # ------------------------------------------------------------------

        self._update_telemetry()

        # ------------------------------------------------------------------
        # Graph
        # ------------------------------------------------------------------

        self._update_joint_plot()

        # ------------------------------------------------------------------
        # Pick/place sequence
        # ------------------------------------------------------------------

        self._update_sequence()

        # ------------------------------------------------------------------
        # Connection indicators
        # ------------------------------------------------------------------

        self._update_connection_status()

    # ======================================================================
    # Arm visualization
    # ======================================================================

    def _update_arm_visualization(self):

        q = list(
            self.node.latest_positions
        )

        try:

            points = self.node.arm.forward_kinematics(
                q
            )

        except Exception as exc:

            self.status_label.setText(
                f"FK error: {exc}"
            )

            return

        if points is None:
            return

        if len(points) != 4:
            return

        # Convert FK result to plain float arrays.
        #
        # Expected:
        #
        # [
        #   (x0, y0),
        #   (x1, y1),
        #   (x2, y2),
        #   (x3, y3)
        # ]

        try:

            xs = [
                float(point[0])
                for point in points
            ]

            ys = [
                float(point[1])
                for point in points
            ]

        except Exception:

            return

        # ------------------------------------------------------------------
        # Arm links
        # ------------------------------------------------------------------

        self.arm_line.setData(
            x=xs,
            y=ys
        )

        # ------------------------------------------------------------------
        # Joint markers
        # ------------------------------------------------------------------

        self.joint_points.setData(
            x=xs,
            y=ys
        )

        # ------------------------------------------------------------------
        # End effector
        # ------------------------------------------------------------------

        self.ee_point.setData(
            x=[xs[-1]],
            y=[ys[-1]]
        )

    # ======================================================================
    # Telemetry
    # ======================================================================

    def _update_telemetry(self):

        q = list(
            self.node.latest_positions
        )

        # ------------------------------------------------------------------
        # Joint angles
        # ------------------------------------------------------------------

        q_deg = [
            math.degrees(
                value
            )
            for value in q
        ]

        self.q1_label.setText(
            f"{q_deg[0]:.2f}°"
        )

        self.q2_label.setText(
            f"{q_deg[1]:.2f}°"
        )

        self.q3_label.setText(
            f"{q_deg[2]:.2f}°"
        )

        # ------------------------------------------------------------------
        # End effector
        # ------------------------------------------------------------------

        try:

            ee = self.node.arm.end_effector(
                q
            )

            self.ee_x_label.setText(
                f"{float(ee[0]):.3f} m"
            )

            self.ee_y_label.setText(
                f"{float(ee[1]):.3f} m"
            )

        except Exception:

            self.ee_x_label.setText(
                "N/A"
            )

            self.ee_y_label.setText(
                "N/A"
            )

        # ------------------------------------------------------------------
        # Joint-state freshness
        # ------------------------------------------------------------------

        age = (
            time.monotonic()
            - self.node.last_joint_state_time
        )

        if age > 1.0:

            self.joint_state_status_label.setText(
                "STALE"
            )

            # Don't overwrite useful controller status
            # if a service operation is active.

            if self.pending_future is None:
                self.status_label.setText(
                    "Waiting for /joint_states"
                )

    # ======================================================================
    # Connection status
    # ======================================================================

    def _update_connection_status(self):

        # Move service
        if self.node.move_client.service_is_ready():

            self.ros_status_label.setText(
                "Connected"
            )

        else:

            self.ros_status_label.setText(
                "Waiting"
            )

        # Joint states
        age = (
            time.monotonic()
            - self.node.last_joint_state_time
        )

        if age < 1.0:

            self.joint_state_status_label.setText(
                "Streaming"
            )

        else:

            self.joint_state_status_label.setText(
                "Waiting"
            )

    # ======================================================================
    # Joint angle plot
    # ======================================================================

    def _update_joint_plot(self):

        elapsed = (
            time.monotonic()
            - self.start_time
        )

        q = list(
            self.node.latest_positions
        )

        # ------------------------------------------------------------------
        # Store current sample
        # ------------------------------------------------------------------

        self.time_history.append(
            float(elapsed)
        )

        self.q1_history.append(
            math.degrees(
                q[0]
            )
        )

        self.q2_history.append(
            math.degrees(
                q[1]
            )
        )

        self.q3_history.append(
            math.degrees(
                q[2]
            )
        )

        # Need at least two points for a visible line
        if len(self.time_history) < 2:
            return

        # ------------------------------------------------------------------
        # Convert deque to lists
        # ------------------------------------------------------------------

        t = list(
            self.time_history
        )

        q1 = list(
            self.q1_history
        )

        q2 = list(
            self.q2_history
        )

        q3 = list(
            self.q3_history
        )

        # ------------------------------------------------------------------
        # Update curves
        # ------------------------------------------------------------------

        self.q1_curve.setData(
            x=t,
            y=q1
        )

        self.q2_curve.setData(
            x=t,
            y=q2
        )

        self.q3_curve.setData(
            x=t,
            y=q3
        )

        # ------------------------------------------------------------------
        # Follow latest 20 seconds
        # ------------------------------------------------------------------

        latest_time = t[-1]

        if latest_time <= PLOT_HISTORY_SECONDS:

            self.joint_plot.setXRange(
                0,
                PLOT_HISTORY_SECONDS,
                padding=0
            )

        else:

            self.joint_plot.setXRange(
                latest_time - PLOT_HISTORY_SECONDS,
                latest_time,
                padding=0
            )

    # ======================================================================
    # Clear plot
    # ======================================================================

    def _clear_plot_history(self):

        self.time_history.clear()
        self.q1_history.clear()
        self.q2_history.clear()
        self.q3_history.clear()

        self.q1_curve.clear()
        self.q2_curve.clear()
        self.q3_curve.clear()

        self.start_time = time.monotonic()

        self.status_label.setText(
            "Joint history cleared"
        )

    # ======================================================================
    # Move commands
    # ======================================================================

    def _move_to_target(self):

        x = self.x_input.value()
        y = self.y_input.value()

        self._send_coordinates(
            x,
            y
        )

    def _move_to_pick(self):

        self._send_coordinates(
            PICK_TARGET[0],
            PICK_TARGET[1]
        )

    def _move_to_place(self):

        self._send_coordinates(
            PLACE_TARGET[0],
            PLACE_TARGET[1]
        )

    def _send_coordinates(
        self,
        x: float,
        y: float
    ):

        # ------------------------------------------------------------------
        # Prevent simultaneous commands
        # ------------------------------------------------------------------

        if self.pending_future is not None:

            if not self.pending_future.done():

                self.status_label.setText(
                    "Waiting for controller response..."
                )

                return

        # ------------------------------------------------------------------
        # Service availability
        # ------------------------------------------------------------------

        if not self.node.move_client.service_is_ready():

            self.status_label.setText(
                "Move service unavailable"
            )

            self.node.get_logger().warning(
                "Move service is not available."
            )

            return

        # ------------------------------------------------------------------
        # Send target
        # ------------------------------------------------------------------

        self.pending_future = (
            self.node.send_target(
                x,
                y
            )
        )

        if self.pending_future is None:

            self.status_label.setText(
                "Failed to send target"
            )

            return

        self.status_label.setText(
            f"Command sent: ({x:.2f}, {y:.2f})"
        )

    # ======================================================================
    # Service response
    # ======================================================================

    def _check_service_response(self):

        if self.pending_future is None:
            return

        if not self.pending_future.done():
            return

        try:

            response = (
                self.pending_future.result()
            )

            if response.success:

                self.status_label.setText(
                    response.message
                )

            else:

                self.status_label.setText(
                    f"Rejected: {response.message}"
                )

                self.node.get_logger().warning(
                    response.message
                )

        except Exception as exc:

            self.status_label.setText(
                f"Service error: {exc}"
            )

        finally:

            self.pending_future = None

    # ======================================================================
    # Pick / Place
    # ======================================================================

    def _start_pick_place(self):

        if self.sequence_active:

            self.status_label.setText(
                "Pick/place sequence already running"
            )

            return

        if not self.node.move_client.service_is_ready():

            self.status_label.setText(
                "Move service unavailable"
            )

            return

        self.sequence_active = True

        self.sequence_step = 0

        self.sequence_wait_start = None

        self.pick_place_button.setEnabled(
            False
        )

        self.move_button.setEnabled(
            False
        )

        self.pick_button.setEnabled(
            False
        )

        self.place_button.setEnabled(
            False
        )

        self.status_label.setText(
            "Pick/place sequence started"
        )

    def _finish_pick_place(self):

        self.sequence_active = False

        self.sequence_step = 0

        self.sequence_wait_start = None

        self.pick_place_button.setEnabled(
            True
        )

        self.move_button.setEnabled(
            True
        )

        self.pick_button.setEnabled(
            True
        )

        self.place_button.setEnabled(
            True
        )

    def _update_sequence(self):

        if not self.sequence_active:
            return

        # Don't send another service request while
        # the current request is still pending.

        if self.pending_future is not None:
            return

        # ------------------------------------------------------------------
        # STEP 0
        # Move to pick
        # ------------------------------------------------------------------

        if self.sequence_step == 0:

            self.status_label.setText(
                "Moving to pick target (4.0, 2.0)"
            )

            self._send_coordinates(
                PICK_TARGET[0],
                PICK_TARGET[1]
            )

            self.sequence_step = 1

            return

        # ------------------------------------------------------------------
        # STEP 1
        # Wait for pick position
        # ------------------------------------------------------------------

        if self.sequence_step == 1:

            if self._at_target(
                PICK_TARGET
            ):

                self.status_label.setText(
                    "Pick position reached"
                )

                self.sequence_wait_start = (
                    time.monotonic()
                )

                self.sequence_step = 2

            return

        # ------------------------------------------------------------------
        # STEP 2
        # Simulated pick operation
        # ------------------------------------------------------------------

        if self.sequence_step == 2:

            if self.sequence_wait_start is None:

                self.sequence_wait_start = (
                    time.monotonic()
                )

                return

            if (
                time.monotonic()
                - self.sequence_wait_start
                >= 0.5
            ):

                self.status_label.setText(
                    "Pick complete"
                )

                self.sequence_step = 3

            return

        # ------------------------------------------------------------------
        # STEP 3
        # Move to place
        # ------------------------------------------------------------------

        if self.sequence_step == 3:

            self.status_label.setText(
                "Moving to place target (3.0, 3.0)"
            )

            self._send_coordinates(
                PLACE_TARGET[0],
                PLACE_TARGET[1]
            )

            self.sequence_step = 4

            return

        # ------------------------------------------------------------------
        # STEP 4
        # Wait for place position
        # ------------------------------------------------------------------

        if self.sequence_step == 4:

            if self._at_target(
                PLACE_TARGET
            ):

                self.status_label.setText(
                    "Place position reached"
                )

                self.sequence_wait_start = (
                    time.monotonic()
                )

                self.sequence_step = 5

            return

        # ------------------------------------------------------------------
        # STEP 5
        # Simulated place operation
        # ------------------------------------------------------------------

        if self.sequence_step == 5:

            if self.sequence_wait_start is None:

                self.sequence_wait_start = (
                    time.monotonic()
                )

                return

            if (
                time.monotonic()
                - self.sequence_wait_start
                >= 0.5
            ):

                self.status_label.setText(
                    "Pick/place sequence complete"
                )

                self._finish_pick_place()

    # ======================================================================
    # Target checking
    # ======================================================================

    def _at_target(
        self,
        target
    ) -> bool:

        try:

            ee = self.node.arm.end_effector(
                self.node.latest_positions
            )

            error = math.hypot(
                float(ee[0]) - float(target[0]),
                float(ee[1]) - float(target[1])
            )

            return error < 0.05

        except Exception:

            return False

    # ======================================================================
    # Window close
    # ======================================================================

    def closeEvent(self, event):

        self.timer.stop()

        event.accept()


# ============================================================================
# Main
# ============================================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    app = QtWidgets.QApplication(
        sys.argv
    )

    node = GuiNode()

    window = MainWindow(
        node
    )

    window.show()

    try:

        exit_code = app.exec_()

    except KeyboardInterrupt:

        exit_code = 0

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(
        exit_code
    )


# ============================================================================
# Entry point
# ============================================================================

if __name__ == "__main__":

    main()