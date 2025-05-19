
---

## 🛠️ Milestone 1: URDF Modeling and Gazebo Simulation

**Goal:** Create a URDF model of a 6-DOF robotic arm and simulate it in Gazebo.

### Tasks:
- Modeled each link and joint using URDF.
- Exported URDF using joint naming convention: `jointX_ID1_ID2`.
- Added joint limits, dynamics, and visuals for simulation.
- Configured `ros2_control` via a `.yaml` file.
- Created a `.launch.py` file to spawn the robot in Gazebo.
- 🎥 [Milestone 1 Video](#)
  
 https://github.com/user-attachments/assets/3f67b7de-71e7-4056-97fe-9b34f08361a5

---

## 🧠 Milestone 2: Motion Planning with MoveIt2

**Goal:** Configure MoveIt2 and demonstrate robot motion in RViz and Gazebo.

### Tasks:
- Created a MoveIt2 configuration package.
- Customized launch files and parameters.
- Demonstrated motion planning to:
  - **Position 0**: Home position (all joints at 0°)
  - **Position 1**: Based on Student A's ID
  - **Position 2**: Based on Student B's ID
- Added gripper open/close positions.
- 🎥 [Milestone 2 Video](#)
  
https://github.com/user-attachments/assets/0ca67aa3-c7d9-4692-854b-6975805250f7

---

## 🧑‍💻 Milestone 3: Python Scripted Control with MoveIt2 + GUI Control

**Goal:** Use Python to control the robotic arm via predefined and custom joint angles.

### Tasks:
- Wrote a Python script using `moveit_py` to:
  - Move to predefined positions (1 & 2)
  - Move to custom positions based on student IDs (3 & 4)
  - Control the gripper
- Printed end-effector pose after motion execution.
- 🎥 [Milestone 3 Video](#)

https://github.com/user-attachments/assets/59b7aa42-4ee0-4e02-8d49-5a0e929987f5

---

## ⚙️ Requirements

To run this project successfully, ensure you have the following installed:

### Operating System
- Ubuntu 24.04 LTS 

### ROS 2
- ROS 2 Jazzy 

### Simulation and Planning Tools
- Gazebo Classic (for simulation)
- RViz2 (for visualization)
- MoveIt2 (for motion planning)

### Python and Libraries
- Python 3.8+
- `moveit_py`
- `rclpy`
- `geometry_msgs`
- `sensor_msgs`
- `PyQt5` or `Tkinter` 

### Build Tools
- colcon
- rosdep

### Optional Tools
- VS Code or any ROS-compatible IDE
- Git (for version control)

### Setup Instructions
Make sure to run the following commands after cloning the repository:
```bash
cd <your_workspace>
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash

