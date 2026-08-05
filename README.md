# Hockey Controller

ROS 2 workspace for running a simple hockey robot mission:

1. Run safe navigation to a target point.
2. Spin in place.
3. Report mission status.

The main launch file starts three core nodes:

- `safe_navigation_server`: action server for goal tracking with CLF-CBF-QP obstacle avoidance.
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
  safe_lookahead_distance:=0.25 \
  rotations:=2 \
  linear_speed:=0.4 \
  angular_speed:=0.8
```

## Safe Navigation Controller

`safe_navigation_server` uses approximate linearization for a unicycle robot.
It controls a point `p` located `lookahead_distance` meters in front of the
robot:

```text
p = [x + l cos(theta), y + l sin(theta)]
p_dot = K * (p_goal - p)
v = cos(theta) * p_dot_x + sin(theta) * p_dot_y
omega = (-sin(theta) * p_dot_x + cos(theta) * p_dot_y) / l
```

The action goal's `linear_speed` and `angular_speed` are maximum speed limits.
The actual `v` and `omega` are computed by the controller.

The planar control point velocity is now computed by a CLF-CBF-QP:

```text
u_nom = K * (p_goal - p)

minimize ||u - u_nom||^2 + w_delta * delta^2

CLF: e^T u <= -clf_gamma * V + delta
CBF: h_dot >= -cbf_gamma * h
```

The CLF slack `delta` can relax goal convergence, but CBF obstacle constraints
are hard. If the QP fails or is infeasible, the robot publishes zero velocity.
The CBF is applied only to the look-ahead point `p`; it does not by itself
guarantee full-body or stick collision avoidance. Use conservative obstacle and
robot safety radii.

The controller uses `qpsolvers` with the `cvxopt` backend:

```bash
python3 -m pip install -r src/hockey_controller/requirements.txt
```

When `orient_to_target` is enabled, safe navigation first drives the control
point `p` to the target pose, then locks into `ORIENT` and only aligns yaw.
It does not return to `TRACK_GOAL` during this final orientation phase.

Example with fixed circular obstacles after launch:

```bash
ros2 param load /safe_navigation_server \
  install/hockey_controller/share/hockey_controller/config/safe_navigation_one_obstacle.yaml
```

Direct safe-navigation action test:

```bash
ros2 action send_goal /safe_navigate_to_point hockey_interfaces/action/NavigateToPoint \
"{target_x: 1.5, target_y: 0.0, linear_speed: 0.3, angular_speed: 0.8, timeout_sec: 20.0}" \
--feedback
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
NAVIGATE
SPIN
MISSION_DONE
```

Stop a running mission:

```bash
ros2 service call /mission/stop std_srvs/srv/Trigger {}
```

After stopping, update the next safe navigation target and restart:

```bash
ros2 param set /mission_manager target_x 1.5
ros2 param set /mission_manager target_y 0.5
ros2 service call /mission/start std_srvs/srv/Trigger {}
```

Safe navigation obstacle parameters can be updated while the node is running:

```bash
ros2 param set /safe_navigation_server obstacle_x "[1.0]"
ros2 param set /safe_navigation_server obstacle_y "[0.0]"
ros2 param set /safe_navigation_server obstacle_radius "[0.15]"
```

## Test Nodes Individually

Spin only:

```bash
ros2 action send_goal /spin hockey_interfaces/action/Spin \
"{rotations: 1, angular_speed: 0.8, timeout_sec: 15.0}" \
--feedback
```

Safe navigation only:

```bash
ros2 action send_goal /safe_navigate_to_point hockey_interfaces/action/NavigateToPoint \
"{target_x: 0.0, target_y: 0.0, linear_speed: 0.3, angular_speed: 0.8, timeout_sec: 20.0}" \
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
