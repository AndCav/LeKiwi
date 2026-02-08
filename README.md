# Ros2 LeKiwi

- [ROS2 Control](https://github.com/ros-controls/ros2_control): generic and simple controls framework
- [MoveIt2](https://github.com/moveit/moveit2): a motion planning, manipulation, and kinematics framework
- [RAI(RobotecAI)](https://github.com/RobotecAI/rai): a flexible AI agent framework to develop and deploy Embodied AI features for your robots

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

## cmd_vel -> wheel velocity pipeline

Enable the cmd_vel translator together with the wheel bridge:

```bash
ros2 launch lekiwi lekiwi_isaac.launch.py \
  use_ros2_control:=false \
  enable_wheel_velocity_bridge:=true \
  enable_cmd_vel_to_wheel:=true
```

Send planar velocity commands:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

Tune kinematics via node parameters if wheel orientation/sign differs:
- `wheel_angles_deg` (default `[0, 120, 240]`)
- `wheel_signs` (default `[1, 1, 1]`)
- `wheel_radius_m` (default `0.05`)
- `robot_radius_m` (default `0.08`)
