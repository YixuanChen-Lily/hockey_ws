import math
from typing import Tuple

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

try:
    from visualization_msgs.msg import Marker, MarkerArray
except ImportError:
    Marker = None
    MarkerArray = None

from hockey_controller.cushion_parking_planner import CushionGeometry
from hockey_controller.cushion_parking_planner import ParkingPlannerConfig
from hockey_controller.cushion_parking_planner import cushion_axes
from hockey_controller.cushion_parking_planner import cushion_radius_layers


def visualization_available() -> bool:
    return Marker is not None and MarkerArray is not None


def marker_array_type():
    return MarkerArray


def publish_parking_markers(
    node,
    publisher,
    frame_id: str,
    robot_pose: Tuple[float, float, float],
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
    plan,
) -> None:
    if not visualization_available():
        node.get_logger().warning(
            "visualization_msgs is unavailable; parking markers disabled"
        )
        return
    if publisher is None:
        return

    markers = MarkerArray()
    delete_marker = Marker()
    delete_marker.action = Marker.DELETEALL
    markers.markers.append(delete_marker)

    marker_id = 0
    markers.markers.append(
        _cube_marker(
            node,
            frame_id,
            marker_id,
            "cushion_body",
            geometry.center_x,
            geometry.center_y,
            geometry.yaw,
            geometry.length,
            geometry.width,
            0.04,
            ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.45),
        )
    )
    marker_id += 1

    radius_layers = _parking_radius_layers(geometry, config)
    for obstacle in plan.cushion_obstacles:
        for namespace, radius, color in radius_layers:
            markers.markers.append(
                _circle_marker(
                    node,
                    frame_id,
                    marker_id,
                    namespace,
                    obstacle.x,
                    obstacle.y,
                    radius,
                    color,
                )
            )
            marker_id += 1

    markers.markers.append(
        _circle_marker(
            node,
            frame_id,
            marker_id,
            "robot_safety_radius",
            robot_pose[0],
            robot_pose[1],
            config.robot_safety_radius,
            ColorRGBA(r=0.0, g=0.8, b=0.25, a=0.28),
        )
    )
    marker_id += 1

    t, n = cushion_axes(geometry)
    markers.markers.append(
        _arrow_marker(
            node,
            frame_id,
            marker_id,
            "front_normal",
            (geometry.center_x, geometry.center_y),
            (
                geometry.center_x + 0.4 * n[0],
                geometry.center_y + 0.4 * n[1],
            ),
            ColorRGBA(r=1.0, g=0.2, b=0.1, a=0.9),
        )
    )
    marker_id += 1
    markers.markers.append(
        _arrow_marker(
            node,
            frame_id,
            marker_id,
            "cushion_local_x",
            (geometry.center_x, geometry.center_y),
            (
                geometry.center_x + 0.4 * t[0],
                geometry.center_y + 0.4 * t[1],
            ),
            ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.9),
        )
    )
    marker_id += 1

    route_points = [(robot_pose[0], robot_pose[1])]
    route_points.extend(plan.waypoints)
    if len(route_points) >= 2:
        markers.markers.append(
            _line_strip_marker(
                node,
                frame_id,
                marker_id,
                "parking_route",
                route_points,
                ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.9),
            )
        )
        marker_id += 1

    labeled_points = [
        (
            "pre_park_point",
            plan.pre_park_point,
            ColorRGBA(r=1.0, g=0.65, b=0.0, a=0.95),
        ),
        (
            "final_park_point",
            plan.final_park_point,
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.95),
        ),
    ]
    if not plan.already_front_side and len(plan.waypoints) > 2:
        for index, waypoint in enumerate(reversed(plan.waypoints[:-2])):
            labeled_points.insert(
                0,
                (
                    f"bypass_waypoint_{len(plan.waypoints[:-2]) - index}",
                    waypoint,
                    ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.95),
                ),
            )
    for namespace, point, color in labeled_points:
        markers.markers.append(
            _sphere_marker(
                node,
                frame_id,
                marker_id,
                namespace,
                point[0],
                point[1],
                0.08,
                color,
            )
        )
        marker_id += 1

    final_heading_end = (
        plan.final_park_point[0] + 0.25 * math.cos(plan.final_yaw),
        plan.final_park_point[1] + 0.25 * math.sin(plan.final_yaw),
    )
    markers.markers.append(
        _arrow_marker(
            node,
            frame_id,
            marker_id,
            "final_yaw",
            plan.final_park_point,
            final_heading_end,
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.9),
        )
    )

    publisher.publish(markers)


def _parking_radius_layers(
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
):
    physical, robot, stick, cbf = cushion_radius_layers(geometry, config)
    layers = [
        (
            "cushion_cbf_radius",
            cbf,
            ColorRGBA(r=0.1, g=0.35, b=1.0, a=0.16),
        )
    ]
    if stick > robot + 1e-9:
        layers.append(
            (
                "cushion_stick_inflated_radius",
                stick,
                ColorRGBA(r=0.45, g=0.15, b=1.0, a=0.18),
            )
        )
    layers.extend(
        [
            (
                "cushion_robot_inflated_radius",
                robot,
                ColorRGBA(r=0.0, g=0.75, b=1.0, a=0.20),
            ),
            (
                "cushion_physical_radius",
                physical,
                ColorRGBA(r=0.9, g=0.55, b=0.05, a=0.26),
            ),
        ]
    )
    unique_layers = []
    previous_radius = None
    for namespace, radius, color in layers:
        if previous_radius is not None and abs(radius - previous_radius) < 1e-9:
            continue
        unique_layers.append((namespace, radius, color))
        previous_radius = radius
    return unique_layers


def _base_marker(node, frame_id: str, marker_id: int, namespace: str, marker_type):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = node.get_clock().now().to_msg()
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _circle_marker(
    node,
    frame_id: str,
    marker_id: int,
    namespace: str,
    x: float,
    y: float,
    radius: float,
    color: ColorRGBA,
):
    marker = _base_marker(node, frame_id, marker_id, namespace, Marker.CYLINDER)
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.01
    marker.scale.x = 2.0 * radius
    marker.scale.y = 2.0 * radius
    marker.scale.z = 0.02
    marker.color = color
    return marker


def _sphere_marker(
    node,
    frame_id: str,
    marker_id: int,
    namespace: str,
    x: float,
    y: float,
    diameter: float,
    color: ColorRGBA,
):
    marker = _base_marker(node, frame_id, marker_id, namespace, Marker.SPHERE)
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.08
    marker.scale.x = diameter
    marker.scale.y = diameter
    marker.scale.z = diameter
    marker.color = color
    return marker


def _cube_marker(
    node,
    frame_id: str,
    marker_id: int,
    namespace: str,
    x: float,
    y: float,
    yaw: float,
    length: float,
    width: float,
    height: float,
    color: ColorRGBA,
):
    marker = _base_marker(node, frame_id, marker_id, namespace, Marker.CUBE)
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.03
    marker.pose.orientation.z = math.sin(0.5 * yaw)
    marker.pose.orientation.w = math.cos(0.5 * yaw)
    marker.scale.x = length
    marker.scale.y = width
    marker.scale.z = height
    marker.color = color
    return marker


def _arrow_marker(
    node,
    frame_id: str,
    marker_id: int,
    namespace: str,
    start: Tuple[float, float],
    end: Tuple[float, float],
    color: ColorRGBA,
):
    marker = _base_marker(node, frame_id, marker_id, namespace, Marker.ARROW)
    start_point = Point()
    start_point.x = start[0]
    start_point.y = start[1]
    start_point.z = 0.08
    end_point = Point()
    end_point.x = end[0]
    end_point.y = end[1]
    end_point.z = 0.08
    marker.points.append(start_point)
    marker.points.append(end_point)
    marker.scale.x = 0.025
    marker.scale.y = 0.055
    marker.scale.z = 0.08
    marker.color = color
    return marker


def _line_strip_marker(
    node,
    frame_id: str,
    marker_id: int,
    namespace: str,
    points: list,
    color: ColorRGBA,
):
    marker = _base_marker(node, frame_id, marker_id, namespace, Marker.LINE_STRIP)
    marker.scale.x = 0.025
    marker.color = color
    for x, y in points:
        point = Point()
        point.x = x
        point.y = y
        point.z = 0.06
        marker.points.append(point)
    return marker
