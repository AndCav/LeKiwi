# omni3ros_pkg (vendored subset)

This directory is a plain third-party vendor copy used by me  to simulate omniwheels on LeKiwi for wheel URDF/mesh assets.

## Origin

- Upstream repository: `https://github.com/YugAjmera/omni3ros_pkg`
- Imported from upstream commit: `3807560d76e0647386a8b216fc2825eed2438fd6`
- Kept in this repo: `urdf/` and `mesh/`

## Original README excerpt

> # omni3ros_pkg
>
> ## A ROS Package for three-wheeled omnidirectional robots
>
> ### Getting Started
>
> - `cd catkin_ws/src`
> - Clone this repo here : `git clone "https://github.com/YugAjmera/omni3ros_pkg"`
> - `cd ..` (Go back to catkin_ws/)
> - `catkin_make`
> - `source ./devel/setup.bash`
> - `source ~/.bashrc`
>
> ### Run
>
> - To view the model in Gazebo : ` roslaunch omni3ros_pkg urdf_gazebo_view.launch `
>
> Model from: [https://github.com/GuiRitter/OpenBase](https://github.com/GuiRitter/OpenBase)
>
> - To view the model with controllers : `roslaunch omni3ros_pkg velocity_controller.launch `
>
> - To view RVIZ model : `roslaunch omni3ros_pkg urdf_rviz_view.launch`
>
> - Control the robot using keyboard keys: [teleop_keyboard_omni3](https://github.com/YugAjmera/teleop_keyboard_omni3)
