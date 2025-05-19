
---

## 🛠️ Milestone 1: URDF Modeling and Gazebo Simulation

**Goal:** Create a URDF model of a 6-DOF robotic arm and simulate it in Gazebo.

### Tasks:
- Modeled each link and joint using URDF.
- Exported URDF using joint naming convention: `jointX_ID1_ID2`.
- Added joint limits, dynamics, and visuals for simulation.
- Configured `ros2_control` via a `.yaml` file.
- Created a `.launch.py` file to spawn the robot in Gazebo.

### Deliverables:
- URDF files (`urdf/`)
- Controller YAML file (`config/`)
- Launch file (`launch/spawn_robot.launch.py`)
- Report with screenshots
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

### Deliverables:
- MoveIt2 config package (`moveit_config/`)
- Launch files for RViz and Gazebo
- Screenshots for each goal pose
- 🎥 [Milestone 2 Video](#)

---

## 🧑‍💻 Milestone 3: Python Scripted Control with MoveIt2

**Goal:** Use Python to control the robotic arm via predefined and custom joint angles.

### Tasks:
- Wrote a Python script using `moveit_py` to:
  - Move to predefined positions (1 & 2)
  - Move to custom positions based on student IDs (3 & 4)
  - Control the gripper
- Printed end-effector pose after motion execution.

### Deliverables:
- Python scripts (`scripts/`)
- Launch files to run simulation and script
- Output screenshots from RViz/Gazebo
- 🎥 [Milestone 3 Video](#)

---

## 👥 Team Members

- **Student A:** Name – ID: `XXXXXX`
- **Student B:** Name – ID: `XXXXXX`

---

## 🧰 Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- MoveIt2
- Python 3.8+

---

## 🚀 How to Launch

### Milestone 1: Spawn robot in Gazebo
```bash
ros2 launch milestone1 spawn_robot.launch.py
