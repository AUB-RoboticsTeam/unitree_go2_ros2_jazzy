<div align="center">

# Unitree Go2 — ROS Noetic Simulation

**The Go2 simulation and junior_ctrl control stack for Ubuntu 20.04, ROS Noetic and Gazebo 11.**

Robot description · `ros_control` · Gazebo worlds · RViz · Unitree L1 lidar · keyboard, joystick, and BLE control

</div>

## Overview

This `noetic` branch is the ROS 1 port of the ROS 2 Jazzy / Gazebo Harmonic
simulation on `main`. It runs the same Go2 with the Unitree L1 lidar, front
camera, IMU and foot contact sensing, the same indoor test worlds, and the same
junior_ctrl state machine driven by the keyboard, a `sensor_msgs/Joy` joystick,
or the Go2 Bluetooth remote.

| Branch | Platform |
| --- | --- |
| `main` | Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic |
| `humble` | ROS 2 Humble |
| `noetic` | Ubuntu 20.04, ROS Noetic, Gazebo 11 (this branch) |

## Repository layout

| Package | Purpose |
| --- | --- |
| `go2_description` | Go2 xacro, meshes and sensors (L1 lidar, camera, IMU, foot contacts, odometry) |
| `unitree_gazebo` | Worlds, `go2_sim.launch`, RViz config, `/odom` publisher, lidar self-filter, foot contact plugins |
| `unitree_legged_control` | Per-joint `ros_control` controller |
| `unitree_legged_msgs` | Unitree low-level command and state messages |
| `unitree_guide` | junior_ctrl state machine and keyboard, joystick or BLE input |
| `velodyne_gazebo_plugins` | GPU lidar -> `PointCloud2` Gazebo plugin (vendored from DataspeedInc/velodyne_simulator 1.0.13, BSD) |

## Requirements

- Ubuntu 20.04 (amd64)
- ROS Noetic Desktop-Full (includes Gazebo 11)
- `ros-noetic-gazebo-ros-control ros-noetic-ros-controllers ros-noetic-joy ros-noetic-topic-tools liblcm-dev`
- Bluetooth adapter and `bluez` only for the Go2 BLE remote

## Quick start

### 1. Clone and install dependencies

```bash
source /opt/ros/noetic/setup.bash

mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
git clone -b noetic https://github.com/AUB-RoboticsTeam/unitree_go2_ros2_jazzy.git

cd ~/go2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
```

### 2. Build the workspace

```bash
conda deactivate 2>/dev/null   # catkin must use the system Python 3.8
cd ~/go2_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

Source `~/go2_ws/devel/setup.bash` in every new terminal that uses this workspace.

### 3. Launch Gazebo and RViz

```bash
roslaunch unitree_gazebo go2_sim.launch
```

Starts the indoor world (`elevation.world`: rooms, furniture, ramp, steps and
obstacles), spawns the Go2, loads its joint controllers and opens RViz with the
L1 point cloud.

## Simulation options

```bash
roslaunch unitree_gazebo go2_sim.launch gui:=false rviz:=false          # headless
roslaunch unitree_gazebo go2_sim.launch world_init_x:=0.0 world_init_y:=0.0 world_init_heading:=1.57
roslaunch unitree_gazebo go2_sim_TI.launch                               # TI-building world and its start pose
roslaunch unitree_gazebo go2_sim.launch world:=/path/to/your.world
```

RViz uses `odom` as its fixed frame; the simulation publishes the dynamic
`odom -> base` transform from Gazebo's ground truth.

## Control the robot

Start the simulation first, then launch junior_ctrl in a second terminal with
one of the three input modes:

```bash
roslaunch unitree_guide junior_ctrl.launch input_mode:=keyboard
roslaunch unitree_guide junior_ctrl.launch input_mode:=joystick
roslaunch unitree_guide junior_ctrl.launch input_mode:=both
```

In `joystick` and `both` modes the launch starts the Go2 BLE remote bridge
(`unitree_address:=98:DA:10:02:9C:4C` by default, `use_unitree_ble:=false` to
disable it). For a USB or Bluetooth gamepad use
`use_unitree_ble:=false use_joy_node:=true joy_dev:=/dev/input/js0`. In `both`
mode the most recently used input controls the robot.

| Key | Action | Key | Action |
| --- | --- | --- | --- |
| `1` | Passive | `2` | Fixed stand |
| `3` | Free stand | `4` | Trotting |
| `5` | move_base (walk from `/cmd_vel`) | `0`, `9`, `8` | Balance, swing and step tests |
| `W` / `S` | Forward / backward | `A` / `D` | Lateral motion |
| `I` / `K` | Pitch input | `J` / `L` | Yaw input |
| `Arrow Up` | Increase speed | `Arrow Down` | Decrease speed |
| `Space` | Reset motion input | | |

Joystick: `B` passive, `A` fixed stand, `X` free stand, `Start` trotting, `Y`
move_base, `L1` + `X`/`A`/`Y` balance/swing/step tests, left stick to move,
right stick to turn, D-pad up/down for speed. The Go2 remote uses buttons `12`
and `14` for the D-pad; a standard joystick's D-pad axis `7` also works. The
maximum linear speed starts at 0.5 m/s and changes in 0.1 m/s steps between
0.1 and 1.2 m/s; the yaw-rate limit scales with it up to 4.5 rad/s.

> [!CAUTION]
> Change states deliberately and keep the simulated robot clear of obstacles
> while testing stand, gait, balance, swing or step modes.

## Main topics

| Topic | Type | Description |
| --- | --- | --- |
| `/unitree_lidar/points` | `sensor_msgs/PointCloud2` | Unitree L1 point cloud (frame `lidar_l1_link`) |
| `/unitree_lidar/points_raw` | `sensor_msgs/PointCloud2` | Same cloud before the robot's own legs are removed |
| `/odom` | `nav_msgs/Odometry` | Gazebo odometry |
| `/tf` | `tf2_msgs/TFMessage` | `odom -> base` and robot transforms |
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz simulated IMU |
| `/trunk_imu` | `sensor_msgs/Imu` | 1 kHz IMU used by junior_ctrl |
| `/rgb_image` | `sensor_msgs/Image` | Front RGB camera |
| `/feet/{FR,FL,RR,RL}/contacts` | `gazebo_msgs/ContactsState` | Foot contacts |
| `/joint_states` | `sensor_msgs/JointState` | Joint states |
| `/joy` | `sensor_msgs/Joy` | Joystick or Unitree BLE remote input |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity command in the move_base state |
| `/estimator/odom` | `nav_msgs/Odometry` | junior_ctrl's own state estimate |

## Differences from the ROS 2 version

- Gazebo 11 uses ODE at a 0.2 ms step (Unitree's tuned settings) instead of Harmonic's physics.
- The robot's root frame is `base` (Unitree's naming) instead of `base_link`.
- The L1 lidar is a GPU ray sensor with the same mount and 600 x 30 scan pattern.
  Gazebo 11 renders the robot's own body, so `lidar_self_filter.py` removes points
  inside the body box (`box_min` / `box_max` parameters).
- Foot contacts use `gazebo_msgs/ContactsState`.

## Troubleshooting

### A launch hangs or the robot does not spawn

A previous `gzserver` is still running. Clean up and launch again:

```bash
rosrun unitree_gazebo clean_sim.sh
```

### The robot will not stand and prints `[SAFETY TRIP]`

It has fallen over; restart the simulation.

### The build picks up the wrong Python

Deactivate conda and rebuild with `-DPYTHON_EXECUTABLE=/usr/bin/python3`.

## Acknowledgements

- [Unitree Robotics — `unitree_ros`](https://github.com/unitreerobotics/unitree_ros) and [`unitree_guide`](https://github.com/unitreerobotics/unitree_guide): robot description, joint controller and junior_ctrl
- [DataspeedInc — `velodyne_simulator`](https://bitbucket.org/DataspeedInc/velodyne_simulator): Gazebo lidar plugin
- **Hadi and the AUB Robotics Team**: the ROS 2 Jazzy simulation this branch is ported from, foot contact sensing, the custom worlds and joystick/BLE integration

## License

Each package keeps its own license (`LICENSE` files and package manifests); the
Unitree-derived packages are BSD 3-Clause.
