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
class DynamicRobotPlotState:
    x: float
    y: float
    timestamp_sec: float


class ParkingPlotter(Node):
    """Standalone matplotlib view for parking radii, route, and trajectory."""

    def __init__(self) -> None:
        super().__init__("parking_plotter")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("marker_topic", "mission/parking_markers")
        self.declare_parameter("plot_rate_hz", 5.0)
        self.declare_parameter("trajectory_length", 400)
        self.declare_parameter("axis_margin", 0.35)
        self.declare_parameter("show_gui", False)
        self.declare_parameter("output_path", "/tmp/parking_plot.png")
        dynamic_ids_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("dynamic_robot_ids", [], dynamic_ids_descriptor)
        self.declare_parameter(
            "dynamic_pose_topic_template",
            "/vrpn_mocap/dji_robot_{robot_id}/pose",
        )
        self.declare_parameter("dynamic_controlled_robot_radius", 0.18)
        self.declare_parameter("dynamic_robot_radius", 0.18)
        self.declare_parameter("dynamic_robot_safety_margin", 0.10)
        self.declare_parameter("dynamic_obstacle_timeout_sec", 0.5)

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
        self.dynamic_robot_ids = self._coerce_int_array(
            self.get_parameter("dynamic_robot_ids").value
        )
        self.dynamic_pose_topic_template = str(
            self.get_parameter("dynamic_pose_topic_template").value
        )
        self.dynamic_obstacle_radius = (
            float(self.get_parameter("dynamic_controlled_robot_radius").value)
            + float(self.get_parameter("dynamic_robot_radius").value)
            + float(self.get_parameter("dynamic_robot_safety_margin").value)
        )
        self.dynamic_obstacle_timeout_sec = float(
            self.get_parameter("dynamic_obstacle_timeout_sec").value
        )
        self.plot_period_sec = 1.0 / max(
            float(self.get_parameter("plot_rate_hz").value),
            0.5,
        )
        trajectory_length = int(self.get_parameter("trajectory_length").value)
        self._trajectory = deque(maxlen=max(1, trajectory_length))
        self._markers: List[Marker] = []
        self._dynamic_robot_states: Dict[int, DynamicRobotPlotState] = {}
        self._dynamic_robot_subscriptions = []
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
        for dynamic_robot_id in self.dynamic_robot_ids:
            topic = self.dynamic_pose_topic_template.format(
                robot_id=dynamic_robot_id
            )
            self._dynamic_robot_subscriptions.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    self._make_dynamic_pose_callback(dynamic_robot_id),
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
            f"  dynamic robots = {self.dynamic_robot_ids}\n"
            f"  dynamic radius = {self.dynamic_obstacle_radius:.2f}\n"
            f"  gui    = {self.show_gui}\n"
            f"  output = {self.output_path}"
        )

    def _coerce_int_array(self, value) -> List[int]:
        if isinstance(value, str):
            parsed = ast.literal_eval(value)
        else:
            parsed = value
        return [int(item) for item in parsed]

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
        with self._lock:
            self._trajectory.append(
                (
                    float(message.pose.position.x),
                    float(message.pose.position.y),
                )
            )
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

    def _make_dynamic_pose_callback(self, robot_id: int):
        def callback(message: PoseStamped) -> None:
            self._dynamic_pose_callback(robot_id, message)

        return callback

    def _dynamic_pose_callback(
        self,
        robot_id: int,
        message: PoseStamped,
    ) -> None:
        with self._lock:
            self._dynamic_robot_states[robot_id] = DynamicRobotPlotState(
                x=float(message.pose.position.x),
                y=float(message.pose.position.y),
                timestamp_sec=time.monotonic(),
            )
            self._dirty = True

    def _draw(self) -> None:
        with self._lock:
            if not self._dirty:
                return
            markers = list(self._markers)
            trajectory = list(self._trajectory)
            dynamic_robot_states = dict(self._dynamic_robot_states)
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
        plotted_points.extend(self._draw_dynamic_obstacles(dynamic_robot_states))

        if trajectory:
            xs = [point[0] for point in trajectory]
            ys = [point[1] for point in trajectory]
            self._axis.plot(xs, ys, color="black", linewidth=1.6, label="robot trajectory")
            self._axis.scatter(xs[-1], ys[-1], color="black", s=35, marker="x", label="robot")
            plotted_points.extend(trajectory)

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

    def _draw_dynamic_obstacles(
        self,
        states: Dict[int, DynamicRobotPlotState],
    ) -> List[Tuple[float, float]]:
        plotted_points = []
        now_sec = time.monotonic()
        for robot_id in self.dynamic_robot_ids:
            state: Optional[DynamicRobotPlotState] = states.get(robot_id)
            if state is None:
                continue
            age_sec = now_sec - state.timestamp_sec
            is_stale = age_sec > self.dynamic_obstacle_timeout_sec
            color = (1.0, 0.0, 0.0, 0.22) if not is_stale else (0.35, 0.35, 0.35, 0.16)
            edge_color = (1.0, 0.0, 0.0) if not is_stale else (0.35, 0.35, 0.35)
            circle = self._patches.Circle(
                (state.x, state.y),
                self.dynamic_obstacle_radius,
                facecolor=color,
                edgecolor=edge_color,
                linewidth=1.4,
                linestyle="-" if not is_stale else "--",
                label=(
                    f"dynamic_obstacle r={self.dynamic_obstacle_radius:.2f}"
                    if not is_stale
                    else f"stale_dynamic_obstacle r={self.dynamic_obstacle_radius:.2f}"
                ),
            )
            self._axis.add_patch(circle)
            self._axis.scatter(
                state.x,
                state.y,
                color=edge_color,
                s=28,
                marker="o",
                label=f"dynamic_robot_{robot_id}",
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
                        state.x - self.dynamic_obstacle_radius,
                        state.y - self.dynamic_obstacle_radius,
                    ),
                    (
                        state.x + self.dynamic_obstacle_radius,
                        state.y + self.dynamic_obstacle_radius,
                    ),
                ]
            )
        return plotted_points

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
