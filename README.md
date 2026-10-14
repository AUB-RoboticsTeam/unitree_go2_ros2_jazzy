# Unitree Go2 ROS 2 Jazzy Simulation

ROS 2 Jazzy packages for simulating and controlling a Unitree Go2 in Gazebo
Harmonic. The repository includes the robot description, Gazebo worlds,
`ros2_control` joint controllers, and `unitree_guide2` for keyboard, joystick,
or Unitree BLE remote control.

## Features

- Ubuntu 24.04, ROS 2 Jazzy, and Gazebo Harmonic
- Go2 URDF/Xacro model and `gz_ros2_control`
- Default, elevation, city, and TI-building worlds
- IMU, RGB camera, Unitree L1 point cloud, and four foot-contact sensors
- Dynamic `odom -> base_link` TF for RViz
- Keyboard, standard ROS `sensor_msgs/Joy`, or combined input
- Direct Unitree Go2 BLE remote to `/joy` bridge
- Bundled ROS 2 Unitree motor messages and joint controller

The active point cloud topic is `/unitree_lidar/points`.

## Repository packages

| Package | Purpose |
| --- | --- |
| `unitree_go2_description` | Go2 Xacro, meshes, sensors, and Gazebo worlds |
| `unitree_go2_sim` | Gazebo/RViz launches and controller configuration |
| `ros2_unitree_legged_msgs` | Unitree low-level ROS 2 messages |
| `ros2_unitree_legged_control` | Per-joint `ros2_control` plugin |
| `unitree_guide2` | Junior controller and keyboard/joystick/BLE input |

## Requirements

- Ubuntu 24.04 (amd64)
- ROS 2 Jazzy Desktop
- Gazebo Harmonic packages supplied with ROS 2 Jazzy
- A Bluetooth adapter only when using the Unitree BLE remote

`rosdep` installs the declared ROS and system dependencies, including BlueZ,
LCM, Eigen, ROS-Gazebo, and ROS 2 Control. The bundled Unitree joint-control
binary is currently built for amd64.

## Installation

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
git clone https://github.com/AUB-RoboticsTeam/unitree_go2_ros2_jazzy.git
cd ~/go2_ws
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install
source install/setup.bash
```

The final `source install/setup.bash` must be repeated in each new terminal, or
added to the shell startup file.

## Run the simulation

Default elevation world with Gazebo and RViz:

```bash
source /opt/ros/jazzy/setup.bash
source ~/go2_ws/install/setup.bash
ros2 launch unitree_go2_sim unitree_go2_launch.py
```

Useful launch options:

```bash
# Headless simulation
ros2 launch unitree_go2_sim unitree_go2_launch.py gui:=false rviz:=false

# Select another included world
ros2 launch unitree_go2_sim unitree_go2_launch.py \
  world:=$(ros2 pkg prefix unitree_go2_description)/share/unitree_go2_description/worlds/city.sdf

# Change the initial pose
ros2 launch unitree_go2_sim unitree_go2_launch.py \
  world_init_x:=0.0 world_init_y:=0.0 world_init_z:=0.5 \
  world_init_heading:=0.0
```

TI-building world using the same active Go2 controller stack:

```bash
ros2 launch unitree_go2_sim unitree_go2_launch_TI.py
```

RViz uses `odom` as its fixed frame. The simulation publishes a dynamic
`odom -> base_link` transform, so the robot moves relative to the RViz grid.

## Control with `unitree_guide2`

Start the simulation first, then run the controller in another terminal.

Keyboard only:

```bash
ros2 launch unitree_guide2 junior_ctrl.launch.py \
  input_mode:=keyboard use_unitree_ble:=false
```

Standard wired/Bluetooth joystick already publishing `/joy`:

```bash
ros2 launch unitree_guide2 junior_ctrl.launch.py \
  input_mode:=joystick use_unitree_ble:=false joy_topic:=/joy
```

Allow both keyboard and joystick input:

```bash
ros2 launch unitree_guide2 junior_ctrl.launch.py \
  input_mode:=both use_unitree_ble:=false joy_topic:=/joy
```

Confirm a standard joystick before starting the controller:

```bash
ros2 topic hz /joy
ros2 topic echo /joy --once
```

### Unitree Go2 BLE remote

Use `bluetoothctl` to discover the controller address:

```bash
bluetoothctl
power on
scan on
devices
pair AA:BB:CC:DD:EE:FF
trust AA:BB:CC:DD:EE:FF
connect AA:BB:CC:DD:EE:FF
quit
```

Then start the bundled BLE bridge and controller:

```bash
ros2 launch unitree_guide2 junior_ctrl.launch.py \
  input_mode:=joystick use_unitree_ble:=true \
  unitree_address:=AA:BB:CC:DD:EE:FF
```

The Unitree remote uses a vendor BLE GATT service rather than the normal Linux
gamepad profile. `unitree_ble_joy` decodes its notifications and publishes
`sensor_msgs/Joy` on `/joy`. It reconnects automatically and publishes neutral
controls if packets become stale.

## Main simulation topics

| Topic | Type | Description |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo 3D odometry |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom -> base_link` and robot transforms |
| `/imu/data` | `sensor_msgs/msg/Imu` | Simulated IMU |
| `/rgb_image` | `sensor_msgs/msg/Image` | Front RGB camera |
| `/unitree_lidar/points` | `sensor_msgs/msg/PointCloud2` | Unitree L1 point cloud |
| `/feet/FR/contacts` | `ros_gz_interfaces/msg/Contacts` | Front-right contact data |
| `/feet/FL/contacts` | `ros_gz_interfaces/msg/Contacts` | Front-left contact data |
| `/feet/RR/contacts` | `ros_gz_interfaces/msg/Contacts` | Rear-right contact data |
| `/feet/RL/contacts` | `ros_gz_interfaces/msg/Contacts` | Rear-left contact data |
| `/joy` | `sensor_msgs/msg/Joy` | Joystick or Unitree BLE remote input |

## Troubleshooting

If an interrupted Gazebo session leaves stale processes, use the included
cleanup helper before relaunching:

```bash
$(ros2 pkg prefix unitree_go2_sim)/lib/unitree_go2_sim/clean_sim.sh
```

Check the odometry TF chain with:

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Check all declared dependencies with:

```bash
cd ~/go2_ws
rosdep check --from-paths src --ignore-src --rosdistro jazzy
```

## Acknowledgements

- [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros)
- [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2)

## License

Individual bundled components retain the licenses declared in their package
manifests and source files. A repository-wide license has not yet been
declared.
