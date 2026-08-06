import math
from dataclasses import dataclass
from typing import List, Tuple

from hockey_controller.clf_cbf_qp import CircularObstacle


@dataclass(frozen=True)
class CushionGeometry:
    center_x: float
    center_y: float
    length: float
    width: float
    yaw: float
    front_axis: str = "y"
    front_normal_sign: float = -1.0


@dataclass(frozen=True)
class ParkingPlannerConfig:
    front_side_threshold: float = 0.0
    side_clearance: float = 0.35
    front_clearance: float = 0.35
    desired_normal_distance: float = 0.35
    tangential_offset: float = 0.0
    parking_lateral_offset: float = 0.0
    pre_park_backoff: float = 0.40
    robot_safety_radius: float = 0.20
    stick_safety_extension: float = 0.0
    safety_margin: float = 0.10
    circle_spacing: float = 0.20
    obstacle_axis: str = "local_x"
    obstacle_radius_override: float = -1.0


@dataclass(frozen=True)
class ParkingPlan:
    already_front_side: bool
    bypass_side: str
    waypoints: Tuple[Tuple[float, float], ...]
    pre_park_point: Tuple[float, float]
    final_park_point: Tuple[float, float]
    final_yaw: float
    cushion_obstacles: Tuple[CircularObstacle, ...]
    message: str = "ok"


def wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def cushion_axes(
    geometry: CushionGeometry,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    t = (math.cos(geometry.yaw), math.sin(geometry.yaw))
    left_normal = (-math.sin(geometry.yaw), math.cos(geometry.yaw))
    sign = 1.0 if geometry.front_normal_sign >= 0.0 else -1.0
    if geometry.front_axis == "x":
        n = (sign * t[0], sign * t[1])
    else:
        n = (sign * left_normal[0], sign * left_normal[1])
    return t, n


def cushion_lateral_axis(geometry: CushionGeometry) -> Tuple[float, float]:
    """Return the positive local axis perpendicular to the selected front axis."""
    t = (math.cos(geometry.yaw), math.sin(geometry.yaw))
    local_y = (-math.sin(geometry.yaw), math.cos(geometry.yaw))
    if geometry.front_axis == "x":
        return local_y
    return t


def classify_front_side(
    robot_position: Tuple[float, float],
    geometry: CushionGeometry,
    front_side_threshold: float,
) -> Tuple[bool, float]:
    _, n = cushion_axes(geometry)
    side_value = (
        n[0] * (robot_position[0] - geometry.center_x)
        + n[1] * (robot_position[1] - geometry.center_y)
    )
    return side_value >= front_side_threshold, side_value


def parking_points(
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
) -> Tuple[Tuple[float, float], Tuple[float, float], float]:
    t, n = cushion_axes(geometry)
    lateral = cushion_lateral_axis(geometry)
    # Positive front normal n points toward the desired parking side.
    # Therefore both final and pre-park points are placed at +n from the
    # cushion center; pre-park is farther out along +n for a straight approach.
    # parking_lateral_offset moves the parking slot along the cushion local
    # axis perpendicular to the selected parking front axis.
    final_x = (
        geometry.center_x
        + config.desired_normal_distance * n[0]
        + config.tangential_offset * t[0]
        + config.parking_lateral_offset * lateral[0]
    )
    final_y = (
        geometry.center_y
        + config.desired_normal_distance * n[1]
        + config.tangential_offset * t[1]
        + config.parking_lateral_offset * lateral[1]
    )
    final_park_point = (final_x, final_y)
    pre_park_point = (
        final_park_point[0] + config.pre_park_backoff * n[0],
        final_park_point[1] + config.pre_park_backoff * n[1],
    )
    final_yaw = wrap_to_pi(math.atan2(-n[1], -n[0]))
    return pre_park_point, final_park_point, final_yaw


def bypass_waypoints(
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
) -> Tuple[
    Tuple[Tuple[float, float], Tuple[float, float]],
    Tuple[Tuple[float, float], Tuple[float, float]],
]:
    axis, axis_length, _ = cushion_obstacle_axis(geometry, config)
    _, n = cushion_axes(geometry)
    negative_endpoint = (
        geometry.center_x - 0.5 * axis_length * axis[0],
        geometry.center_y - 0.5 * axis_length * axis[1],
    )
    positive_endpoint = (
        geometry.center_x + 0.5 * axis_length * axis[0],
        geometry.center_y + 0.5 * axis_length * axis[1],
    )
    negative_route = _bypass_route_around_endpoint(
        negative_endpoint,
        -1.0,
        axis,
        n,
        config,
    )
    positive_route = _bypass_route_around_endpoint(
        positive_endpoint,
        1.0,
        axis,
        n,
        config,
    )
    return negative_route, positive_route


def _bypass_route_around_endpoint(
    endpoint: Tuple[float, float],
    side_sign: float,
    axis: Tuple[float, float],
    n: Tuple[float, float],
    config: ParkingPlannerConfig,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    outside_endpoint = (
        endpoint[0] + side_sign * config.side_clearance * axis[0],
        endpoint[1] + side_sign * config.side_clearance * axis[1],
    )
    back_side_waypoint = (
        outside_endpoint[0] - config.front_clearance * n[0],
        outside_endpoint[1] - config.front_clearance * n[1],
    )
    front_side_waypoint = (
        outside_endpoint[0] + config.front_clearance * n[0],
        outside_endpoint[1] + config.front_clearance * n[1],
    )
    return back_side_waypoint, front_side_waypoint


def cushion_cbf_circles(
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
) -> Tuple[CircularObstacle, ...]:
    axis, axis_length, _ = cushion_obstacle_axis(geometry, config)
    radius = cushion_cbf_radius(geometry, config)
    spacing = max(config.circle_spacing, 1e-3)
    count = max(2, int(math.ceil(axis_length / spacing)) + 1)
    circles: List[CircularObstacle] = []
    for index in range(count):
        alpha = -0.5 + index / (count - 1)
        offset = alpha * axis_length
        circles.append(
            CircularObstacle(
                geometry.center_x + offset * axis[0],
                geometry.center_y + offset * axis[1],
                radius,
            )
        )
    return tuple(circles)


def cushion_radius_layers(
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
) -> Tuple[float, float, float, float]:
    _, _, cross_width = cushion_obstacle_axis(geometry, config)
    physical_radius = 0.5 * cross_width
    robot_inflated_radius = physical_radius + config.robot_safety_radius
    stick_inflated_radius = robot_inflated_radius + config.stick_safety_extension
    cbf_radius = cushion_cbf_radius(geometry, config)
    return (
        physical_radius,
        robot_inflated_radius,
        stick_inflated_radius,
        cbf_radius,
    )


def cushion_cbf_radius(
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
) -> float:
    if config.obstacle_radius_override > 0.0:
        return config.obstacle_radius_override
    _, _, cross_width = cushion_obstacle_axis(geometry, config)
    return (
        0.5 * cross_width
        + config.robot_safety_radius
        + config.stick_safety_extension
        + config.safety_margin
    )


def cushion_obstacle_axis(
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
) -> Tuple[Tuple[float, float], float, float]:
    t = (math.cos(geometry.yaw), math.sin(geometry.yaw))
    local_y = (-math.sin(geometry.yaw), math.cos(geometry.yaw))
    if config.obstacle_axis in ("local_y", "y"):
        return local_y, geometry.width, geometry.length
    return t, geometry.length, geometry.width


def plan_parking_route(
    robot_position: Tuple[float, float],
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
) -> ParkingPlan:
    if geometry.length <= 0.0 or geometry.width <= 0.0:
        return ParkingPlan(
            False,
            "none",
            (),
            (math.nan, math.nan),
            (math.nan, math.nan),
            math.nan,
            (),
            "invalid cushion geometry",
        )

    pre_park_point, final_park_point, final_yaw = parking_points(
        geometry,
        config,
    )
    cushion_obstacles = cushion_cbf_circles(geometry, config)
    is_front, _ = classify_front_side(
        robot_position,
        geometry,
        config.front_side_threshold,
    )
    if is_front:
        return ParkingPlan(
            True,
            "none",
            (pre_park_point, final_park_point),
            pre_park_point,
            final_park_point,
            final_yaw,
            cushion_obstacles,
            "already on parking side",
        )

    left_route, right_route = bypass_waypoints(geometry, config)
    candidates = (("left", left_route), ("right", right_route))
    valid_candidates = []
    for side, route in candidates:
        cost = (
            _distance(robot_position, route[0])
            + _distance(route[0], route[1])
            + _distance(route[1], pre_park_point)
        )
        valid_candidates.append((cost, side, route))

    if not valid_candidates:
        return ParkingPlan(
            False,
            "none",
            (),
            pre_park_point,
            final_park_point,
            final_yaw,
            cushion_obstacles,
            "no valid bypass route",
        )

    _, side, route = min(valid_candidates, key=lambda item: item[0])
    return ParkingPlan(
        False,
        side,
        tuple(route) + (pre_park_point, final_park_point),
        pre_park_point,
        final_park_point,
        final_yaw,
        cushion_obstacles,
        "bypass selected",
    )

def _distance(
    first: Tuple[float, float],
    second: Tuple[float, float],
) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])
