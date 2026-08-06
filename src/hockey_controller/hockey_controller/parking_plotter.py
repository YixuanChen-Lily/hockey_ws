#!/usr/bin/env python3

import ast
import math
import time
from collections import deque
from dataclasses import dataclass
from threading import Lock
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class ObstacleRobotPlotState:
    x: float
    y: float
    timestamp_sec: float


@dataclass(frozen=True)
class PoseObstaclePlotSpec:
    key: str
    topic: str
    radius: float


@dataclass
class PosePlotState:
    x: float
    y: float
    yaw: float
    timestamp_sec: float


class ParkingPlotter(Node):
    def __init__(self) -> None:
        super().__init__("parking_plotter")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("marker_topic", "mission/parking_markers")
        self.declare_parameter("plot_rate_hz", 5.0)
        self.declare_parameter("trajectory_length", 400)
        self.declare_parameter("axis_margin", 0.35)
        self.declare_parameter("show_gui", False)
        self.declare_parameter(
            "output_path",
            "/hockey_ws/src/hockey_controller/parking_plot.png",
        )
        self.declare_parameter("show_shooting_geometry", True)
        self.declare_parameter("puck_pose_topic", "/vrpn_mocap/puck/pose")
        self.declare_parameter("goal_pose_topic", "/vrpn_mocap/goal/pose")
        self.declare_parameter("shooting_role", "shooter")
        self.declare_parameter("shooting_offset_x", 0.0)
        self.declare_parameter("shooting_offset_y", 0.0)
        self.declare_parameter("shooting_target_radius", 0.20)
        self.declare_parameter("shooting_contact_gap", 0.0)
        self.declare_parameter("shooting_center_to_puck_distance", -1.0)
        self.declare_parameter("shooting_spin_direction", "ccw")
        self.declare_parameter("shooting_puck_obstacle_enabled", True)
        self.declare_parameter("shooting_puck_obstacle_radius", 0.10)
        self.declare_parameter("safe_lookahead_distance", 0.25)
        self.declare_parameter("pose_timeout_sec", 150.0)
        obstacle_ids_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("obstacle_robot_ids", [], obstacle_ids_descriptor)
        obstacle_topics_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("obstacle_pose_topics", [], obstacle_topics_descriptor)
        self.declare_parameter("obstacle_pose_radii", [], obstacle_ids_descriptor)
        self.declare_parameter(
            "obstacle_pose_topic_template",
            "/vrpn_mocap/dji_robot_{robot_id}/pose",
        )
        self.declare_parameter("obstacle_controlled_robot_radius", 0.0)
        self.declare_parameter("obstacle_robot_radius", 0.18)
        self.declare_parameter("obstacle_robot_safety_margin", 0.0)
        self.declare_parameter("obstacle_pose_timeout_sec", 150.0)

        robot_id = int(self.get_parameter("robot_id").value)
        pose_topic = str(self.get_parameter("pose_topic").value)
        self.pose_topic = (
            pose_topic
            if pose_topic
            else f"/vrpn_mocap/dji_robot_{robot_id}/pose"
        )
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.axis_margin = float(self.get_parameter("axis_margin").value)
        self.show_gui = bool(self.get_parameter("show_gui").value)
        self.output_path = str(self.get_parameter("output_path").value)
        self.show_shooting_geometry = bool(
            self.get_parameter("show_shooting_geometry").value
        )
        self.puck_pose_topic = str(self.get_parameter("puck_pose_topic").value)
        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.shooting_role = str(self.get_parameter("shooting_role").value)
        self.shooting_offset_x = float(
            self.get_parameter("shooting_offset_x").value
        )
        self.shooting_offset_y = float(
            self.get_parameter("shooting_offset_y").value
        )
        self.shooting_target_radius = float(
            self.get_parameter("shooting_target_radius").value
        )
        self.shooting_contact_gap = float(
            self.get_parameter("shooting_contact_gap").value
        )
        self.shooting_center_to_puck_distance = float(
            self.get_parameter("shooting_center_to_puck_distance").value
        )
        self.shooting_spin_direction = str(
            self.get_parameter("shooting_spin_direction").value
        )
        self.shooting_puck_obstacle_enabled = bool(
            self.get_parameter("shooting_puck_obstacle_enabled").value
        )
        self.shooting_puck_obstacle_radius = float(
            self.get_parameter("shooting_puck_obstacle_radius").value
        )
        self.safe_lookahead_distance = float(
            self.get_parameter("safe_lookahead_distance").value
        )
        self.pose_timeout_sec = float(self.get_parameter("pose_timeout_sec").value)
        self.obstacle_robot_ids = self._coerce_int_array(
            self.get_parameter("obstacle_robot_ids").value
        )
        self.obstacle_pose_topics = self._coerce_string_array(
            self.get_parameter("obstacle_pose_topics").value
        )
        self.obstacle_pose_radii = self._coerce_float_array(
            self.get_parameter("obstacle_pose_radii").value
        )
        if len(self.obstacle_pose_radii) != len(self.obstacle_pose_topics):
            self.obstacle_pose_radii = [
                self.shooting_puck_obstacle_radius
                for _ in self.obstacle_pose_topics
            ]
        self.obstacle_pose_topic_template = str(
            self.get_parameter("obstacle_pose_topic_template").value
        )
        self.obstacle_pose_radius = (
            float(self.get_parameter("obstacle_controlled_robot_radius").value)
            + float(self.get_parameter("obstacle_robot_radius").value)
            + float(self.get_parameter("obstacle_robot_safety_margin").value)
        )
        self.obstacle_pose_safety_margin = float(
            self.get_parameter("obstacle_robot_safety_margin").value
        )
        self.obstacle_pose_timeout_sec = float(
            self.get_parameter("obstacle_pose_timeout_sec").value
        )
        self.plot_period_sec = 1.0 / max(
            float(self.get_parameter("plot_rate_hz").value),
            0.5,
        )
        trajectory_length = int(self.get_parameter("trajectory_length").value)
        self._trajectory = deque(maxlen=max(1, trajectory_length))
        self._robot_pose: Optional[PosePlotState] = None
        self._markers: List[Marker] = []
        self._puck_pose: Optional[PosePlotState] = None
        self._goal_pose: Optional[PosePlotState] = None
        self._obstacle_robot_states: Dict[int, ObstacleRobotPlotState] = {}
        self._pose_obstacle_states: Dict[str, PosePlotState] = {}
        self._pose_obstacle_specs: List[PoseObstaclePlotSpec] = []
        self._obstacle_robot_subscriptions = []
        self._pose_obstacle_subscriptions = []
        self._dirty = True
        self._lock = Lock()
        self._callback_group = ReentrantCallbackGroup()

        self._pose_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._marker_subscription = self.create_subscription(
            MarkerArray,
            self.marker_topic,
            self._marker_callback,
            10,
            callback_group=self._callback_group,
        )
        self._puck_subscription = self.create_subscription(
            PoseStamped,
            self.puck_pose_topic,
            self._puck_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        self._goal_subscription = self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self._goal_pose_callback,
            qos_profile_sensor_data,
            callback_group=self._callback_group,
        )
        for obstacle_robot_id in self.obstacle_robot_ids:
            topic = self.obstacle_pose_topic_template.format(
                robot_id=obstacle_robot_id
            )
            self._obstacle_robot_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    self._make_obstacle_pose_callback(obstacle_robot_id),
                    qos_profile_sensor_data,
                    callback_group=self._callback_group,
                )
            )
        for index, topic in enumerate(self.obstacle_pose_topics):
            spec = PoseObstaclePlotSpec(
                key=f"pose_obstacle_{index}",
                topic=topic,
                radius=(
                    self.obstacle_pose_radii[index]
                    + self.obstacle_pose_safety_margin
                ),
            )
            self._pose_obstacle_specs.append(spec)
            self._pose_obstacle_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda message, spec=spec: (
                        self._pose_obstacle_callback(spec, message)
                    ),
                    qos_profile_sensor_data,
                    callback_group=self._callback_group,
                )
            )

        self._plt, self._patches, self._transforms = self._import_matplotlib()
        self._figure = self._plt.figure(figsize=(12.0, 6.5))
        grid = self._figure.add_gridspec(1, 2, width_ratios=[3.2, 1.2])
        self._axis = self._figure.add_subplot(grid[0, 0])
        self._legend_axis = self._figure.add_subplot(grid[0, 1])
        self._legend_axis.axis("off")
        if self.show_gui:
            self._figure.canvas.manager.set_window_title("Parking Radius Plot")

        self.get_logger().info(
            "Parking plotter ready:\n"
            f"  pose   = {self.pose_topic}\n"
            f"  marker = {self.marker_topic}\n"
            f"  puck   = {self.puck_pose_topic}\n"
            f"  goal   = {self.goal_pose_topic}\n"
            f"  obstacle robots = {self.obstacle_robot_ids}\n"
            f"  obstacle radius = {self.obstacle_pose_radius:.2f}\n"
            f"  pose obstacles = "
            f"{[(spec.topic, spec.radius) for spec in self._pose_obstacle_specs]}\n"
            f"  gui    = {self.show_gui}\n"
            f"  output = {self.output_path}"
        )

    def _coerce_int_array(self, value) -> List[int]:
        if isinstance(value, str):
            parsed = ast.literal_eval(value)
        else:
            parsed = value
        return [int(item) for item in parsed]

    def _coerce_float_array(self, value) -> List[float]:
        if isinstance(value, str):
            parsed = ast.literal_eval(value)
        else:
            parsed = value
        return [float(item) for item in parsed]

    def _coerce_string_array(self, value) -> List[str]:
        if isinstance(value, str):
            parsed = ast.literal_eval(value)
        else:
            parsed = value
        return [str(item) for item in parsed]

    def _import_matplotlib(self):
        try:
            if not self.show_gui:
                import matplotlib

                matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches
            import matplotlib.transforms as transforms
        except ImportError as exception:
            raise RuntimeError(
                "matplotlib is required for parking_plotter. "
                "Install it in Docker with: apt update && "
                "apt install -y python3-matplotlib"
            ) from exception
        if self.show_gui:
            plt.ion()
        return plt, patches, transforms

    def _pose_callback(self, message: PoseStamped) -> None:
        pose = self._pose_state_from_message(message)
        with self._lock:
            self._robot_pose = pose
            self._trajectory.append((pose.x, pose.y))
            self._dirty = True

    def _marker_callback(self, message: MarkerArray) -> None:
        with self._lock:
            for marker in message.markers:
                if marker.action == Marker.DELETEALL:
                    self._markers = []
                    continue
                if marker.action == Marker.DELETE:
                    self._markers = [
                        existing
                        for existing in self._markers
                        if not (
                            existing.ns == marker.ns
                            and existing.id == marker.id
                        )
                    ]
                    continue
                self._markers = [
                    existing
                    for existing in self._markers
                    if not (
                        existing.ns == marker.ns
                        and existing.id == marker.id
                    )
                ]
                self._markers.append(marker)
            self._dirty = True

    def _make_obstacle_pose_callback(self, robot_id: int):
        def callback(message: PoseStamped) -> None:
            self._obstacle_pose_callback(robot_id, message)

        return callback

    def _obstacle_pose_callback(
        self,
        robot_id: int,
        message: PoseStamped,
    ) -> None:
        with self._lock:
            self._obstacle_robot_states[robot_id] = ObstacleRobotPlotState(
                x=float(message.pose.position.x),
                y=float(message.pose.position.y),
                timestamp_sec=time.monotonic(),
            )
            self._dirty = True

    def _pose_obstacle_callback(
        self,
        spec: PoseObstaclePlotSpec,
        message: PoseStamped,
    ) -> None:
        with self._lock:
            self._pose_obstacle_states[spec.key] = self._pose_state_from_message(
                message
            )
            self._dirty = True

    def _puck_pose_callback(self, message: PoseStamped) -> None:
        with self._lock:
            self._puck_pose = self._pose_state_from_message(message)
            self._dirty = True

    def _goal_pose_callback(self, message: PoseStamped) -> None:
        with self._lock:
            self._goal_pose = self._pose_state_from_message(message)
            self._dirty = True

    def _pose_state_from_message(self, message: PoseStamped) -> PosePlotState:
        return PosePlotState(
            x=float(message.pose.position.x),
            y=float(message.pose.position.y),
            yaw=self._yaw_from_quaternion(message.pose.orientation),
            timestamp_sec=time.monotonic(),
        )

    def _draw(self) -> None:
        with self._lock:
            if not self._dirty:
                return
            markers = list(self._markers)
            trajectory = list(self._trajectory)
            robot_pose = self._robot_pose
            obstacle_robot_states = dict(self._obstacle_robot_states)
            pose_obstacle_states = dict(self._pose_obstacle_states)
            puck_pose = self._puck_pose
            goal_pose = self._goal_pose
            self._dirty = False

        self._axis.clear()
        self._legend_axis.clear()
        self._legend_axis.axis("off")
        self._axis.set_title("Parking Obstacles, Safety Radius, and Trajectory")
        self._axis.set_xlabel("x [m]")
        self._axis.set_ylabel("y [m]")
        self._axis.grid(True)
        self._axis.set_aspect("equal", adjustable="box")

        plotted_points: List[Tuple[float, float]] = []
        for marker in markers:
            plotted_points.extend(self._draw_marker(marker))
        plotted_points.extend(self._draw_pose_obstacles(obstacle_robot_states))
        plotted_points.extend(self._draw_generic_pose_obstacles(pose_obstacle_states))
        plotted_points.extend(self._draw_shooting_geometry(puck_pose, goal_pose))

        if trajectory:
            xs = [point[0] for point in trajectory]
            ys = [point[1] for point in trajectory]
            self._axis.plot(xs, ys, color="black", linewidth=1.6, label="robot trajectory")
            self._axis.scatter(xs[-1], ys[-1], color="black", s=35, marker="x", label="robot")
            plotted_points.extend(trajectory)
        if robot_pose is not None:
            plotted_points.extend(self._draw_virtual_point(robot_pose))

        self._set_axis_limits(plotted_points)
        self._dedupe_legend()
        if self.show_gui:
            self._figure.canvas.draw_idle()
        else:
            self._figure.savefig(self.output_path, dpi=130, bbox_inches="tight")

    def _draw_marker(self, marker: Marker) -> List[Tuple[float, float]]:
        if marker.type == Marker.CYLINDER:
            return self._draw_circle(marker)
        if marker.type == Marker.CUBE:
            return self._draw_cube(marker)
        if marker.type == Marker.SPHERE:
            return self._draw_sphere(marker)
        if marker.type == Marker.LINE_STRIP:
            points = [(point.x, point.y) for point in marker.points]
            if points:
                xs = [point[0] for point in points]
                ys = [point[1] for point in points]
                self._axis.plot(xs, ys, color="magenta", linewidth=2.0, label=marker.ns)
            return points
        if marker.type == Marker.ARROW and len(marker.points) >= 2:
            start = marker.points[0]
            end = marker.points[1]
            color = self._rgba(marker)
            self._axis.arrow(
                start.x,
                start.y,
                end.x - start.x,
                end.y - start.y,
                color=color,
                head_width=0.05,
                length_includes_head=True,
                label=marker.ns,
            )
            return [(start.x, start.y), (end.x, end.y)]
        return []

    def _draw_pose_obstacles(
        self,
        states: Dict[int, ObstacleRobotPlotState],
    ) -> List[Tuple[float, float]]:
        plotted_points = []
        now_sec = time.monotonic()
        for robot_id in self.obstacle_robot_ids:
            state: Optional[ObstacleRobotPlotState] = states.get(robot_id)
            if state is None:
                continue
            age_sec = now_sec - state.timestamp_sec
            is_stale = age_sec > self.obstacle_pose_timeout_sec
            color = (1.0, 0.0, 0.0, 0.22) if not is_stale else (0.35, 0.35, 0.35, 0.16)
            edge_color = (1.0, 0.0, 0.0) if not is_stale else (0.35, 0.35, 0.35)
            circle = self._patches.Circle(
                (state.x, state.y),
                self.obstacle_pose_radius,
                facecolor=color,
                edgecolor=edge_color,
                linewidth=1.4,
                linestyle="-" if not is_stale else "--",
                label=(
                    f"robot obstacle r={self.obstacle_pose_radius:.2f}"
                    if not is_stale
                    else f"stale robot obstacle r={self.obstacle_pose_radius:.2f}"
                ),
            )
            self._axis.add_patch(circle)
            self._axis.scatter(
                state.x,
                state.y,
                color=edge_color,
                s=28,
                marker="o",
                label=f"obstacle_robot_{robot_id}",
            )
            self._axis.text(
                state.x,
                state.y,
                str(robot_id),
                color=edge_color,
                fontsize=8,
                ha="center",
                va="center",
            )
            plotted_points.extend(
                [
                    (
                        state.x - self.obstacle_pose_radius,
                        state.y - self.obstacle_pose_radius,
                    ),
                    (
                        state.x + self.obstacle_pose_radius,
                        state.y + self.obstacle_pose_radius,
                    ),
                ]
            )
        return plotted_points

    def _draw_generic_pose_obstacles(
        self,
        states: Dict[str, PosePlotState],
    ) -> List[Tuple[float, float]]:
        plotted_points = []
        now_sec = time.monotonic()
        for index, spec in enumerate(self._pose_obstacle_specs):
            state = states.get(spec.key)
            if state is None:
                continue
            age_sec = now_sec - state.timestamp_sec
            is_stale = age_sec > self.obstacle_pose_timeout_sec
            color = (1.0, 0.2, 0.2, 0.20) if not is_stale else (0.35, 0.35, 0.35, 0.14)
            edge_color = "tab:red" if not is_stale else "0.35"
            circle = self._patches.Circle(
                (state.x, state.y),
                spec.radius,
                facecolor=color,
                edgecolor=edge_color,
                linewidth=1.4,
                linestyle="-" if not is_stale else "--",
                label=(
                    f"pose topic obstacle r={spec.radius:.2f}"
                    if index == 0
                    else "_nolegend_"
                ),
            )
            self._axis.add_patch(circle)
            self._axis.scatter(
                state.x,
                state.y,
                color=edge_color,
                s=30,
                marker="o",
                label=f"pose_obs_{index}",
            )
            self._axis.text(
                state.x,
                state.y,
                f"O{index}",
                color=edge_color,
                fontsize=8,
                ha="center",
                va="center",
            )
            plotted_points.extend(
                [
                    (state.x - spec.radius, state.y - spec.radius),
                    (state.x + spec.radius, state.y + spec.radius),
                ]
            )
        return plotted_points

    def _draw_virtual_point(
        self,
        robot_pose: PosePlotState,
    ) -> List[Tuple[float, float]]:
        now_sec = time.monotonic()
        if now_sec - robot_pose.timestamp_sec > self.pose_timeout_sec:
            return []
        virtual_x = (
            robot_pose.x + self.safe_lookahead_distance * math.cos(robot_pose.yaw)
        )
        virtual_y = (
            robot_pose.y + self.safe_lookahead_distance * math.sin(robot_pose.yaw)
        )
        self._axis.plot(
            [robot_pose.x, virtual_x],
            [robot_pose.y, virtual_y],
            color="tab:cyan",
            linewidth=1.8,
            label="virtual point offset",
        )
        self._axis.scatter(
            virtual_x,
            virtual_y,
            color="tab:cyan",
            s=55,
            marker="D",
            label="virtual point",
        )
        self._axis.text(
            virtual_x,
            virtual_y,
            "V",
            color="tab:cyan",
            fontsize=9,
            ha="left",
            va="bottom",
        )
        return [(robot_pose.x, robot_pose.y), (virtual_x, virtual_y)]

    def _draw_shooting_geometry(
        self,
        puck_pose: Optional[PosePlotState],
        goal_pose: Optional[PosePlotState],
    ) -> List[Tuple[float, float]]:
        if not self.show_shooting_geometry:
            return []
        now_sec = time.monotonic()
        puck_is_fresh = (
            puck_pose is not None
            and now_sec - puck_pose.timestamp_sec <= self.pose_timeout_sec
        )
        goal_is_fresh = (
            goal_pose is not None
            and now_sec - goal_pose.timestamp_sec <= self.pose_timeout_sec
        )
        if not puck_is_fresh and not goal_is_fresh:
            return []
        if not puck_is_fresh or not goal_is_fresh:
            return self._draw_available_puck_goal(
                puck_pose if puck_is_fresh else None,
                goal_pose if goal_is_fresh else None,
            )
        assert puck_pose is not None
        assert goal_pose is not None

        target_x = goal_pose.x
        target_y = goal_pose.y
        if self.shooting_role in ("passer", "single"):
            cos_goal = math.cos(goal_pose.yaw)
            sin_goal = math.sin(goal_pose.yaw)
            target_x += (
                self.shooting_offset_x * cos_goal
                - self.shooting_offset_y * sin_goal
            )
            target_y += (
                self.shooting_offset_x * sin_goal
                + self.shooting_offset_y * cos_goal
            )

        dx = target_x - puck_pose.x
        dy = target_y - puck_pose.y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return [(puck_pose.x, puck_pose.y), (target_x, target_y)]

        ux = dx / distance
        uy = dy / distance
        if self.shooting_spin_direction == "ccw":
            normal_x = uy
            normal_y = -ux
        else:
            normal_x = -uy
            normal_y = ux
        side_distance = self._center_to_puck_distance()
        robot_x = puck_pose.x - side_distance * normal_x
        robot_y = puck_pose.y - side_distance * normal_y
        heading_x = -ux
        heading_y = -uy
        tip_x = robot_x + self.safe_lookahead_distance * heading_x
        tip_y = robot_y + self.safe_lookahead_distance * heading_y

        self._axis.plot(
            [puck_pose.x, target_x],
            [puck_pose.y, target_y],
            color="tab:green",
            linewidth=2.0,
            label="shoot line: puck-target",
        )
        self._axis.plot(
            [robot_x, puck_pose.x],
            [robot_y, puck_pose.y],
            color="tab:purple",
            linewidth=1.8,
            linestyle=":",
            label="perpendicular: center-puck",
        )
        self._axis.plot(
            [robot_x, tip_x],
            [robot_y, tip_y],
            color="tab:olive",
            linewidth=2.0,
            linestyle="--",
            label="lookahead: center-to-stick-tip",
        )
        target_circle = self._patches.Circle(
            (target_x, target_y),
            self.shooting_target_radius,
            facecolor=(0.0, 0.6, 0.0, 0.10),
            edgecolor="tab:green",
            linewidth=1.4,
            linestyle="--",
            label=f"shoot target r={self.shooting_target_radius:.2f}",
        )
        self._axis.add_patch(target_circle)
        self._axis.scatter(
            [robot_x, tip_x, puck_pose.x, target_x],
            [robot_y, tip_y, puck_pose.y, target_y],
            color=["black", "tab:orange", "tab:blue", "tab:green"],
            s=[45, 55, 55, 55],
            label="shooting geometry points",
        )

        self._axis.text(robot_x, robot_y, "R*", color="black", fontsize=9)
        self._axis.text(tip_x, tip_y, "T", color="tab:orange", fontsize=9)
        self._axis.text(puck_pose.x, puck_pose.y, "P", color="tab:blue", fontsize=9)
        self._axis.text(target_x, target_y, "G*", color="tab:green", fontsize=9)
        if self.shooting_puck_obstacle_enabled:
            puck_obstacle = self._patches.Circle(
                (puck_pose.x, puck_pose.y),
                self.shooting_puck_obstacle_radius,
                facecolor=(0.1, 0.3, 1.0, 0.08),
                edgecolor="tab:blue",
                linewidth=1.4,
                linestyle="--",
                label=f"puck obstacle r={self.shooting_puck_obstacle_radius:.2f}",
            )
            self._axis.add_patch(puck_obstacle)
        self._axis.arrow(
            robot_x,
            robot_y,
            0.20 * heading_x,
            0.20 * heading_y,
            color="tab:olive",
            head_width=0.04,
            length_includes_head=True,
            label="robot heading",
        )

        return [
            (robot_x, robot_y),
            (tip_x, tip_y),
            (
                puck_pose.x - self.shooting_puck_obstacle_radius,
                puck_pose.y - self.shooting_puck_obstacle_radius,
            ),
            (
                puck_pose.x + self.shooting_puck_obstacle_radius,
                puck_pose.y + self.shooting_puck_obstacle_radius,
            ),
            (target_x - self.shooting_target_radius, target_y - self.shooting_target_radius),
            (target_x + self.shooting_target_radius, target_y + self.shooting_target_radius),
        ]

    def _draw_available_puck_goal(
        self,
        puck_pose: Optional[PosePlotState],
        goal_pose: Optional[PosePlotState],
    ) -> List[Tuple[float, float]]:
        plotted_points = []
        if puck_pose is not None:
            self._axis.scatter(
                puck_pose.x,
                puck_pose.y,
                color="tab:blue",
                s=55,
                marker="o",
                label="puck",
            )
            self._axis.text(
                puck_pose.x,
                puck_pose.y,
                "P",
                color="tab:blue",
                fontsize=9,
            )
            plotted_points.append((puck_pose.x, puck_pose.y))
        if goal_pose is not None:
            self._axis.scatter(
                goal_pose.x,
                goal_pose.y,
                color="tab:green",
                s=55,
                marker="o",
                label="goal",
            )
            self._axis.text(
                goal_pose.x,
                goal_pose.y,
                "G",
                color="tab:green",
                fontsize=9,
            )
            plotted_points.append((goal_pose.x, goal_pose.y))
        return plotted_points

    def _center_to_puck_distance(self) -> float:
        self.shooting_center_to_puck_distance = float(
            self.get_parameter("shooting_center_to_puck_distance").value
        )
        if self.shooting_center_to_puck_distance > 0.0:
            return self.shooting_center_to_puck_distance
        return self.safe_lookahead_distance + max(0.0, self.shooting_contact_gap)

    def _draw_circle(self, marker: Marker) -> List[Tuple[float, float]]:
        x = marker.pose.position.x
        y = marker.pose.position.y
        radius = 0.5 * max(marker.scale.x, marker.scale.y)
        color = self._rgba(marker)
        circle = self._patches.Circle(
            (x, y),
            radius,
            facecolor=color,
            edgecolor=color[:3],
            linewidth=1.2,
            label=f"{marker.ns} r={radius:.2f}",
        )
        self._axis.add_patch(circle)
        return [(x - radius, y - radius), (x + radius, y + radius)]

    def _draw_cube(self, marker: Marker) -> List[Tuple[float, float]]:
        x = marker.pose.position.x
        y = marker.pose.position.y
        yaw = self._yaw_from_marker(marker)
        length = marker.scale.x
        width = marker.scale.y
        color = self._rgba(marker)
        rectangle = self._patches.Rectangle(
            (-0.5 * length, -0.5 * width),
            length,
            width,
            angle=0.0,
            facecolor=color,
            edgecolor=color[:3],
            linewidth=1.5,
            label=marker.ns,
        )
        transform = (
            self._transforms.Affine2D()
            .rotate(yaw)
            .translate(x, y)
            + self._axis.transData
        )
        rectangle.set_transform(transform)
        self._axis.add_patch(rectangle)
        return [(x - length, y - width), (x + length, y + width)]

    def _draw_sphere(self, marker: Marker) -> List[Tuple[float, float]]:
        x = marker.pose.position.x
        y = marker.pose.position.y
        color = self._rgba(marker)
        self._axis.scatter(x, y, color=color, s=70, label=marker.ns)
        return [(x, y)]

    def _rgba(self, marker: Marker):
        return (
            marker.color.r,
            marker.color.g,
            marker.color.b,
            marker.color.a,
        )

    def _yaw_from_marker(self, marker: Marker) -> float:
        z = marker.pose.orientation.z
        w = marker.pose.orientation.w
        return 2.0 * math.atan2(z, w)

    def _yaw_from_quaternion(self, quaternion) -> float:
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def _set_axis_limits(self, points: List[Tuple[float, float]]) -> None:
        if not points:
            self._axis.set_xlim(-2.5, 2.5)
            self._axis.set_ylim(-2.5, 2.5)
            return
        xs = [point[0] for point in points]
        ys = [point[1] for point in points]
        margin = self.axis_margin
        self._axis.set_xlim(min(xs) - margin, max(xs) + margin)
        self._axis.set_ylim(min(ys) - margin, max(ys) + margin)

    def _dedupe_legend(self) -> None:
        handles, labels = self._axis.get_legend_handles_labels()
        unique = {}
        for handle, label in zip(handles, labels):
            unique.setdefault(label, handle)
        if unique:
            self._legend_axis.legend(
                unique.values(),
                unique.keys(),
                loc="upper left",
                frameon=True,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParkingPlotter()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node._draw()
            if node.show_gui:
                node._plt.pause(node.plot_period_sec)
            else:
                time.sleep(node.plot_period_sec)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
