# Hockey Controller

ROS 2 workspace for running a simple hockey robot mission:

1. Navigate to a target point.
2. Run safe navigation to the final target.
3. Spin in place.
4. Report mission status.

The main launch file starts four nodes:

- `navigation_server`: action server for driving to a goal.
- `safe_navigation_server`: action server for conservative final navigation.
- `spin_server`: action server for spinning in place.
- `mission_manager`: runs the mission steps and publishes status.

## Prerequisites

- ROS 2 installed and sourced.
- `colcon` installed.
- VRPN pose topic available for the robot.

Default topics for `robot_id:=1`:

```text
/vrpn_mocap/dji_robot_1/pose
/robot1/cmd_vel
```

## Build

From the workspace root:

```bash
cd /hockey_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select hockey_interfaces hockey_controller --symlink-install
source install/setup.bash
```

Check that ROS sees the package from the correct workspace:

```bash
ros2 pkg prefix hockey_controller
```

Expected output:

```text
/hockey_ws/install/hockey_controller
```

## Launch

Start the controller stack:

```bash
ros2 launch hockey_controller mission.launch.py
```

With custom parameters:

```bash
ros2 launch hockey_controller mission.launch.py \
  robot_id:=1 \
  target_x:=1.5 \
  target_y:=0.5 \
  safe_target_x:=1.5 \
  safe_target_y:=0.5 \
  rotations:=2 \
  linear_speed:=0.4 \
  angular_speed:=0.8
```

## Start the Mission

In another terminal:

```bash
cd /hockey_ws
source /opt/ros/$ROS_DISTRO/setup.bash
source install/setup.bash
ros2 service call /mission/start std_srvs/srv/Trigger {}
```

Watch mission status:

```bash
ros2 topic echo /mission/status
```

Expected status sequence:

```text
STEP1_NAVIGATE
STEP2_SAFE_NAVIGATE
STEP3_SPIN
MISSION_DONE
```

## Test Nodes Individually

Navigation only:

```bash
ros2 action send_goal /navigate_to_point hockey_interfaces/action/NavigateToPoint \
"{target_x: 1.0, target_y: 0.0, linear_speed: 0.3, angular_speed: 0.8, timeout_sec: 20.0}" \
--feedback
```

Spin only:

```bash
ros2 action send_goal /spin hockey_interfaces/action/Spin \
"{rotations: 1, angular_speed: 0.8, timeout_sec: 15.0}" \
--feedback
```

## Troubleshooting

If `mission.launch.py` is not found, rebuild and source from the workspace root:

```bash
cd /hockey_ws
rm -rf build install log
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select hockey_interfaces hockey_controller --symlink-install
source install/setup.bash
ros2 launch hockey_controller mission.launch.py
```

If the robot does not move and feedback shows `WAIT_FOR_POSE`, check the VRPN pose:

```bash
ros2 topic echo /vrpn_mocap/dji_robot_1/pose
```

If using a different robot, pass the matching ID:

```bash
ros2 launch hockey_controller mission.launch.py robot_id:=2
```




ros2 action send_goal /safe_navigate_to_point hockey_interfaces/action/NavigateToPoint \
"{target_x: 0.0, target_y: 0.0, linear_speed: 0.3, angular_speed: 0.8, timeout_sec: 20.0}" \
--feedback