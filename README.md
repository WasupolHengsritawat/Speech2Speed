# Speech2Speed

A ROS 2–based interface for converting **speech commands into speed control signals** for mobile robots.

This project enables natural language velocity control of robots to `geometry_msgs/Twist` commands published through ROS 2 services or topics. It is useful for human-robot interaction (HRI), assistive robotics, and multimodal teleoperation systems.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Repository Structure](#repository-structure)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [ROS 2 Interface](#ros-2-interface)
- [Examples](#examples)
- [Development Notes](#development-notes)
- [License](#license)

---

## Overview

**Speech2Speed** allows users to control robot velocity using natural language input.  
It listens to a speech stream or receives preprocessed text commands, then interprets them into structured twist trajectories (`linear.x`, `angular.z`) for robot motion.

It is designed to integrate with other ROS 2 packages and can serve as an intelligent interface node within a larger behavior or control framework.

---

## Features

✅ Speech-to-speed translation using simple command parsing  
✅ ROS 2 service for sending velocity trajectories  
✅ Configurable linear/angular scaling  
✅ Example interface for simulation and testing  
✅ Extensible structure for integrating NLP or ASR back-ends (e.g. Whisper, Vosk, or Google Speech API)

---

## Repository Structure

```
Speech2Speed/
├── speech2speed/                 # Core logic for command-to-speed mapping
├── speech2speed_interface/       # ROS 2 interface node (service definitions, publishers)
├── CMakeLists.txt                # Build configuration for ROS 2 packages
├── package.xml                   # ROS 2 package manifest
└── README.md                     # This file
```

---

## Requirements

- **ROS 2 Humble** (or later)
- **Python 3.8+**
- Packages:
  - `rclpy`
  - `geometry_msgs`

*(Additional dependencies may be listed in `requirements.txt` or import statements.)*

---

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/WasupolHengsritawat/Speech2Speed.git
   cd Speech2Speed
   ```

2. Build the package in your ROS 2 workspace:
   ```bash
   colcon build --packages-select speech2speed speech2speed_interface
   ```

3. Source your workspace:
   ```bash
   source install/setup.bash
   ```

4. (Optional) Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

---

## Usage

### Run the ROS 2 Node

To launch the interface node that listens for twist trajectories:
```bash
ros2 launch speech2speed speech2speed.launch.py
```

### Send a Twist Trajectory via Service

You can send a trajectory using a custom ROS 2 service call:
```bash
ros2 service call /Prompt speech2speed_interface/srv/String "prompt: 'Call ROS2 service named "/Traj" to send a linear velocity trajectory of w_z start at w_z=0.0 rad/s and end with w_z=12.0 rad/s at 25 Hz for 2 seconds'"
```

This sends a list of velocity commands that can be used to drive a robot or simulation.

---

## ROS 2 Interface

| Type     | Name             | Description |
|-----------|------------------|--------------|
| **Service** | `/Traj` | Accepts `speech2speed_interface/srv/TwistTraj` to apply or simulate velocity trajectories |
| **Message** | `geometry_msgs/Twist` | Used internally for velocity representation |

---

## Examples

### 1. Constant Speed Function
```python
from speech2speed import constant_func

v = constant_func(0.5)
print(v(10))  # Output: 0.5 (constant)
```

### 2. Linear Speed Function
```python
from speech2speed import linear_func

v = linear_func(start=0.0, slope=0.1, duration=10)
print(v(5))  # Output: 0.5
```

### 3. Speech Command to Speed Mapping
```python
from speech2speed import parse_command

cmd = "move faster and turn right"
twist = parse_command(cmd)
print(twist.linear.x, twist.angular.z)
```

---

## Development Notes

- Extend `speech2speed/` to integrate an **ASR/NLP pipeline**.
- Modify scaling parameters for different robot models.
- You can integrate this with a **Behavior Tree** or **navigation stack** for adaptive control.
- The trajectory format can be adapted to support acceleration or smoothing profiles.

---

## License

This project is released under the **MIT License**.  
See the [LICENSE](LICENSE) file for details.

---

### Author

**Wasupol Hengsritawat**  
📧 [GitHub Profile](https://github.com/WasupolHengsritawat)

---

> _Speech2Speed – Speak. Command. Move._