# Ros2 LeKiwi

- [ROS2 Control](https://github.com/ros-controls/ros2_control): generic and simple controls framework
- [MoveIt2](https://github.com/moveit/moveit2): a motion planning, manipulation, and kinematics framework
- [RAI(RobotecAI)](https://github.com/RobotecAI/rai): a flexible AI agent framework to develop and deploy Embodied AI features for your robots

## Omni3 wheel assets (vendored third-party)

LeKiwi vendors the omni wheel assets from upstream `omni3ros_pkg` as a normal third-party folder (no submodule, no `.git` metadata):

- path: `third_party/omni3ros_pkg`
- upstream: `https://github.com/YugAjmera/omni3ros_pkg`
- usage in URDF: `package://lekiwi/third_party/omni3ros_pkg/mesh/...`
- local attribution + upstream excerpt: `third_party/omni3ros_pkg/README.md`

Only the asset subset used by LeKiwi is kept there (wheel URDF/mesh files).

## Isaac wheel velocity control (clean start)

Launch Isaac + LeKiwi with wheel bridge enabled and ros2_control disabled:

```bash
ros2 launch lekiwi lekiwi_isaac.launch.py use_ros2_control:=false enable_wheel_velocity_bridge:=true
```

Publish wheel velocities (rad/s) in order `[joint7, joint8, joint9]`:

```bash
ros2 topic pub -r 20 /lekiwi/wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [3.0, -3.0, 1.5]}"
```

Quick checks:

```bash
ros2 topic echo /isaac_joint_commands_wheels
ros2 topic echo /topic_based_joint_states
```

Direct wheel spin without ROS wheel commands:

```bash
ros2 launch lekiwi lekiwi_isaac.launch.py \
  use_ros2_control:=false \
  enable_wheel_velocity_bridge:=false \
  wheel_direct_velocity:=3.0,3.0,3.0
```

## ROS2 wheel velocity controller

Enable `ros2_control` and the bridge, then publish velocity commands to the wheel controller:

```bash
ros2 launch lekiwi lekiwi_isaac.launch.py use_ros2_control:=true enable_wheel_velocity_bridge:=true
```

Check controller state:

```bash
ros2 control list_controllers -c /lekiwi/controller_manager
```

Command topic (`std_msgs/msg/Float64MultiArray`, rad/s, order `[joint7, joint8, joint9]`):

```bash
ros2 topic pub -r 20 /lekiwi/wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [3.0, 3.0, 3.0]}"
```

## Arm and gripper control (`JointTrajectory`)

Use `ros2_control` mode so the mux/controller pipeline is active:

```bash
ros2 launch lekiwi lekiwi_isaac.launch.py use_ros2_control:=true enable_wheel_velocity_bridge:=true
```

Arm command topic:

- topic: `/lekiwi/arm_position_controller/joint_trajectory`
- type: `trajectory_msgs/msg/JointTrajectory`

Joint names recognized by LeKiwi:

- `STS3215_03a_v1_Revolute_45`
- `STS3215_03a_v1_1_Revolute_49`
- `STS3215_03a_v1_2_Revolute_51`
- `STS3215_03a_v1_3_Revolute_53`
- `STS3215_03a_Wrist_Roll_v1_Revolute_55`
- `STS3215_03a_v1_4_Revolute_57`

Arm trajectory rules:

- if `joint_names` contains an invalid name, the command is rejected
- if `joint_names` is omitted, the first `n` arm joints are controlled from `positions`, with `1 <= n <= 5`

Example with explicit arm joint names:

```bash
ros2 topic pub --once /lekiwi/arm_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['STS3215_03a_v1_Revolute_45','STS3215_03a_v1_1_Revolute_49','STS3215_03a_v1_2_Revolute_51','STS3215_03a_v1_3_Revolute_53','STS3215_03a_Wrist_Roll_v1_Revolute_55'],
  points: [{positions: [0.0, -0.5, 0.8, 0.3, 0.0], time_from_start: {sec: 1, nanosec: 0}}]
}"
```

Gripper command topic:

- topic: `/lekiwi/gripper_position_controller/joint_trajectory`
- type: `trajectory_msgs/msg/JointTrajectory`

Example gripper command:

```bash
ros2 topic pub --once /lekiwi/gripper_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['STS3215_03a_v1_4_Revolute_57'],
  points: [{positions: [0.6], time_from_start: {sec: 0, nanosec: 200000000}}]
}"
```

## cmd_vel -> wheel velocity pipeline

Enable the cmd_vel translator together with the wheel bridge:

```bash
ros2 launch lekiwi lekiwi_isaac.launch.py \
  use_ros2_control:=false \
  enable_wheel_velocity_bridge:=true \
  enable_cmd_vel_to_wheel:=true
```

By default, the translator listens on `/lekiwi/cmd_vel` to isolate LeKiwi from unrelated global `/cmd_vel` publishers.
If you want the legacy behavior, set `cmd_vel_topic:=/cmd_vel` at launch.
By default, startup brake is disabled (`startup_brake_seconds:=0.0`) so the robot does not start in pause.
If you need startup drift suppression, set `startup_brake_seconds:=2.0` (or higher).

Quick diagnostics for command sources:

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /lekiwi/cmd_vel -v
ros2 topic echo /lekiwi/wheel_velocity_controller/commands
```

Kinematic convention used by the translator:

- ROS frame: `+x` forward, `+y` left, `+omega_z` counterclockwise
- alpha reference: `alpha=0` on `+x`, positive counterclockwise
- formula per wheel:
  - `v_w = v_x * sin(alpha) + v_y * cos(alpha) + omega * R`
  - `omega_wheel = (v_w / r) * sign`
- command array order is fixed: `[joint7, joint8, joint9]`

### Quick wheel-to-joint calibration (single-wheel spin test)

Run these one by one and visually note which physical wheel spins:

```bash
ros2 topic pub -r 10 /lekiwi/wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [2.0, 0.0, 0.0]}"
ros2 topic pub -r 10 /lekiwi/wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 2.0, 0.0]}"
ros2 topic pub -r 10 /lekiwi/wheel_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 2.0]}"
```

With current LeKiwi URDF layout this maps to:

- `joint7` -> wheel1 (top-left/front-left) -> `alpha=60°` -> `sign=+1`
- `joint8` -> wheel2 (rear) -> `alpha=180°` -> `sign=+1`
- `joint9` -> wheel3 (top-right/front-right) -> `alpha=300°` -> `sign=+1`

So default translator parameters are:

- `cmd_vel_topic:=/lekiwi/cmd_vel`
- `cmd_vel_wheel_angles_deg:=[60.0, 180.0, 300.0]`
- `cmd_vel_wheel_signs:=[1.0, 1.0, 1.0]`
- `cmd_vel_wheel_radius_m:=0.05`
- `cmd_vel_robot_radius_m:=0.125`
- `cmd_vel_linear_deadband_m_s:=0.01`
- `cmd_vel_angular_deadband_rad_s:=0.05`
- `startup_brake_seconds:=0.0`

You can override these at launch:

```bash
ros2 launch lekiwi lekiwi_isaac.launch.py \
  use_ros2_control:=false \
  enable_wheel_velocity_bridge:=true \
  enable_cmd_vel_to_wheel:=true \
  cmd_vel_topic:=/lekiwi/cmd_vel \
  cmd_vel_wheel_angles_deg:='[60.0, 180.0, 300.0]' \
  cmd_vel_wheel_signs:='[1.0, 1.0, 1.0]' \
  cmd_vel_wheel_radius_m:=0.05 \
  cmd_vel_robot_radius_m:=0.125 \
  cmd_vel_linear_deadband_m_s:=0.01 \
  cmd_vel_angular_deadband_rad_s:=0.05 \
  startup_brake_seconds:=0.0
```

Send planar velocity commands:

```bash
ros2 topic pub -r 20 /lekiwi/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

Global-frame RViz checks:

- `vx > 0` -> mostly forward motion (+x)
- `vy > 0` -> mostly left motion (+y)
- `wz > 0` -> mostly counterclockwise yaw

If one direction is inverted, tune `cmd_vel_wheel_signs` first.

## Wheel model swap

Wheel control pipeline is unchanged (`cmd_vel -> wheel commands -> Isaac bridge`), but wheel collision geometry is now based on omni3 rim + passive rollers.

Joint mapping remains:

- `joint7` -> top-left wheel
- `joint8` -> rear wheel
- `joint9` -> top-right wheel

Kinematic defaults remain:

- `wheel_radius_m=0.05`
- `robot_radius_m=0.125`
