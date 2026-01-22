# corgi_sim - Corgi leg-wheel Robot Webots Simulation

ROS2 simulation package for the Corgi leg-wheel robot using Webots and `webots_ros2_driver`.

---

## Overview

This package provides a Webots simulation environment for the Corgi robot using the `webots_ros2_driver` plugin architecture.

**Features:**
- ✅ **Python driver plugin** - `corgi_driver_pkg.corgi_driver.CorgiDriver` runs as ROS2 plugin
- ✅ **Theta-beta coordinate system** - Custom leg control for 2-motor-per-leg design
- ✅ **ROS2 topic interface** - Standard `/motor/command` and `/motor/state` topics
- ✅ **Simulation time sync** - `/clock` topic for time-accurate control
- ✅ **TF broadcasting** - Publishes robot pose in `odom` frame
- ✅ **IMU simulation** - Publishes orientation and acceleration data
---

## Architecture

```
Webots Simulator (IFS_Proto.wbt)
  └─ CorgiRobot (controller = "<extern>", supervisor = TRUE)
      ↕
webots_ros2_driver (ROS2 node)
  ├─ Loads: corgi_driver_pkg.corgi_driver.CorgiDriver
  ├─ Publishes:
  │   • /clock (rosgraph_msgs/Clock) - Simulation time
  │   • /motor/state (corgi_msgs/MotorStateStamped) - Motor feedback in θ-β
  │   • /imu (corgi_msgs/ImuStamped) - IMU data
  │   • /tf (odom → base_link transform)
  └─ Subscribes:
      • /motor/command (corgi_msgs/MotorCmdStamped) - Motor commands in θ-β
      ↕
Your Controller Node (e.g., corgi_csv_control)
  └─ Publishes: /motor/command
```

---

## Prerequisites

```bash
# Install Webots ROS2 driver
sudo apt update
sudo apt install ros-humble-webots-ros2

# Ensure corgi_msgs is built first
cd ~/corgi_ws/corgi_ros2_ws
colcon build --packages-select corgi_msgs
```

---

## Quick Start

### 1. Build the Package

```bash
cd ~/corgi_ws/corgi_ros2_ws
colcon build --packages-select corgi_sim
source install/setup.bash
```

### 2. Launch Simulation

```bash
ros2 launch corgi_sim Corgi_launch.py
```

### 3. Run a Controller

In a separate terminal:

```bash
source ~/corgi_ws/corgi_ros2_ws/install/setup.bash

# Example: CSV trajectory playback
ros2 run corgi_csv_control corgi_csv_control demo_walk_sim \
  --ros-args -p use_sim_time:=true
```

**Important:** Always add `-p use_sim_time:=true` to sync with simulation clock!

---

**Motor naming convention:**

- Module A (LF - Left Front): A_L_Motor (left), A_R_Motor (right)
- Module B (RF - Right Front): B_L_Motor (left), B_R_Motor (right)
- Module C (RH - Rear Right): C_L_Motor (left), C_R_Motor (right)
- Module D (LH - Left Rear): D_L_Motor (left), D_R_Motor (right)

## ROS2 Topics

### Published by CorgiDriver

- `/clock` (rosgraph_msgs/Clock) - Simulation time
- `/motor/state` (corgi_msgs/MotorStateStamped) - Motor positions (θ, β), velocities, torques
- `/imu` (corgi_msgs/ImuStamped) - Orientation (quaternion), angular velocity, linear acceleration
- `/tf` (tf2_msgs/TFMessage) - Transform: `odom → base_link`

### Subscribed by CorgiDriver

- `/motor/command` (corgi_msgs/MotorCmdStamped) - Desired θ, β, PID gains (kp, kd), feedforward torques

---

## Package Structure

```
corgi_sim/
├── CMakeLists.txt          # Minimal build config (Python only)
├── package.xml             # Dependencies: webots_ros2_driver, corgi_msgs
├── launch/
│   └── Corgi_launch.py     # Main launch file
├── protos/
│   └── CorgiRobot.proto    # Webots robot definition
├── resource/
│   └── corgi.urdf          # Plugin reference
└── corgi_driver_pkg/       # Python driver package
    ├── __init__.py
    ├── corgi_driver.py     # CorgiDriver plugin class
    └── Controller_TB.py    # θ-β conversion utilities
```

## Troubleshooting

### ModuleNotFoundError: 'corgi_driver_pkg'

```bash
# Rebuild package
cd ~/corgi_ws/corgi_ros2_ws
colcon build --packages-select corgi_sim
source install/setup.bash

# Verify installation
python3 -c "import corgi_driver_pkg.corgi_driver"
```

### Robot doesn't respond to commands

Check topics:
```bash
# Verify driver is publishing
ros2 topic echo /motor/state

# Verify controller is publishing
ros2 topic echo /motor/command

# Check time sync
ros2 topic echo /clock
```

### Controller runs too fast/slow

Ensure `use_sim_time:=true`:
```bash
ros2 run your_pkg your_node --ros-args -p use_sim_time:=true

# Verify parameter
ros2 param get /your_node use_sim_time
# Should return: Boolean value is: True
```

### Motors produce NaN or invalid values

Check command values:
- `theta >= theta_0` (minimum 0.2967 rad)
- All gains are positive
- Torques are within ±35 N·m (Max_Torque limit)

---

## Default Parameters

**PID Gains (in CorgiDriver):**
```python
KP = 1000.0         # Position proportional gain
KD = 1.0            # Velocity derivative gain  
Max_Torque = 35.0   # Maximum motor torque (N·m)
```
