# Kineshia Robotics — ROS 2 Manipulator Take-Home (Starter Repo)

Welcome, and thanks for taking the time. This repo is your starting point for
the **Robotics Software Engineer (ROS 2 & GUI Systems)** take-home task. The
full brief (what to build, how you're evaluated, how to submit) is in the
separate task document you were sent. This README covers setup only.

---

## What's in here

```
kineshia_ros2_arm_task/
├── README.md                         <- you are here (setup only)
└── src/
    └── planar_arm_control/           <- an ament_python package to build on
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/planar_arm_control
        ├── launch/
        │   └── bringup.launch.py      <- STUB: launch controller + GUI
        └── planar_arm_control/
            ├── planar_arm.py          <- PROVIDED kinematics. DO NOT MODIFY.
            ├── controller_node.py     <- STUB: your controller
            └── gui_node.py            <- STUB: your PyQt5/PyQtGraph GUI
```

**`planar_arm.py` is given to you fully working** — forward kinematics,
multiple-solution analytical inverse kinematics (with a damped-Jacobian
fallback), joint limits, and a ground constraint. Treat it as a black-box
library and build around it. Please don't modify it; if you think it has a
bug, note it in your write-up instead.

The three other files are **stubs** with `TODO`s describing the interface we
suggest. You implement them.

---

## The arm

A 3-DoF planar (2D) revolute arm.

| Property        | Value                                  |
|-----------------|----------------------------------------|
| Link lengths    | `[3.0, 2.0, 1.5]`                      |
| Joint 1 limits  | `0°` to `180°`                         |
| Joint 2 limits  | `-120°` to `120°`                      |
| Joint 3 limits  | `-120°` to `120°`                      |
| Constraint      | no part of the arm may go below `y = 0`|

Quick sanity check of the provided library:

```bash
python3 src/planar_arm_control/planar_arm_control/planar_arm.py
```

---

## Environment

- **ROS 2 Jazzy** (preferred) or **Humble** on **Ubuntu**.
- Python 3, `numpy`.
- GUI dependencies: `PyQt5` and `pyqtgraph`
  ```bash
  pip install PyQt5 pyqtgraph        # or: sudo apt install python3-pyqt5 python3-pyqtgraph
  ```

## Build & run

```bash
# from the repo root (this is your colcon workspace root)
colcon build
source install/setup.bash

# once you've implemented the nodes:
ros2 launch planar_arm_control bringup.launch.py
```

---

## What to do next

Open the **task brief** for the full requirements, the core vs. stretch split,
the deliverables (repo + design note + short demo recording), and the timeline.
If anything is unclear, email **hr@kineshia.in** — reasonable questions are
welcome and won't count against you.

Good luck — we're excited to see how you architect it.
