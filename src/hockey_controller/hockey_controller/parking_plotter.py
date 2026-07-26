#!/usr/bin/env python3

import math
import time
from collections import deque
from threading import Lock
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray


class ParkingPlotter(Node):
    """Standalone matplotlib view for parking radii, route, and trajectory."""

    def __init__(self) -> None:
        super().__init__("parking_plotter")

        self.declare_parameter("robot_id", 1)
        self.declare_parameter("pose_topic", "")
        self.declare_parameter("marker_topic", "/mission/parking_markers")
        self.declare_parameter("plot_rate_hz", 5.0)
        self.declare_parameter("trajectory_length", 400)
        self.declare_parameter("axis_margin", 0.35)
        self.declare_parameter("show_gui", False)
        self.declare_parameter("output_path", "/tmp/parking_plot.png")

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
        self.plot_period_sec = 1.0 / max(
            float(self.get_parameter("plot_rate_hz").value),
            0.5,
        )
        trajectory_length = int(self.get_parameter("trajectory_length").value)
        self._trajectory = deque(maxlen=max(1, trajectory_length))
        self._markers: List[Marker] = []
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

        self._plt, self._patches, self._transforms = self._import_matplotlib()
        self._figure, self._axis = self._plt.subplots()
        if self.show_gui:
            self._figure.canvas.manager.set_window_title("Parking Radius Plot")

        self.get_logger().info(
            "Parking plotter ready:\n"
            f"  pose   = {self.pose_topic}\n"
            f"  marker = {self.marker_topic}\n"
            f"  gui    = {self.show_gui}\n"
            f"  output = {self.output_path}"
        )

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

    def _draw(self) -> None:
        with self._lock:
            if not self._dirty:
                return
            markers = list(self._markers)
            trajectory = list(self._trajectory)
            self._dirty = False

        self._axis.clear()
        self._axis.set_title("Parking Obstacles, Safety Radius, and Trajectory")
        self._axis.set_xlabel("x [m]")
        self._axis.set_ylabel("y [m]")
        self._axis.grid(True)
        self._axis.set_aspect("equal", adjustable="box")

        plotted_points: List[Tuple[float, float]] = []
        for marker in markers:
            plotted_points.extend(self._draw_marker(marker))

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
            self._figure.savefig(self.output_path, dpi=130)

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
            self._axis.legend(unique.values(), unique.keys(), loc="upper right")


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
