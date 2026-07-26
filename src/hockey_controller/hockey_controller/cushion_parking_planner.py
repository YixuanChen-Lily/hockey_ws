import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

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
    pre_park_backoff: float = 0.40
    robot_safety_radius: float = 0.20
    stick_safety_extension: float = 0.0
    safety_margin: float = 0.10
    circle_spacing: float = 0.20
    field_min_x: Optional[float] = None
    field_max_x: Optional[float] = None
    field_min_y: Optional[float] = None
    field_max_y: Optional[float] = None


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


def cushion_endpoints(
    geometry: CushionGeometry,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    t, _ = cushion_axes(geometry)
    half_length = 0.5 * geometry.length
    left_endpoint = (
        geometry.center_x - half_length * t[0],
        geometry.center_y - half_length * t[1],
    )
    right_endpoint = (
        geometry.center_x + half_length * t[0],
        geometry.center_y + half_length * t[1],
    )
    return left_endpoint, right_endpoint


def parking_points(
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
) -> Tuple[Tuple[float, float], Tuple[float, float], float]:
    t, n = cushion_axes(geometry)
    # Positive front normal n points toward the desired parking side.
    # Therefore both final and pre-park points are placed at +n from the
    # cushion center; pre-park is farther out along +n for a straight approach.
    final_park_point = (
        geometry.center_x
        + config.desired_normal_distance * n[0]
        + config.tangential_offset * t[0],
        geometry.center_y
        + config.desired_normal_distance * n[1]
        + config.tangential_offset * t[1],
    )
    pre_park_point = (
        final_park_point[0] + config.pre_park_backoff * n[0],
        final_park_point[1] + config.pre_park_backoff * n[1],
    )
    final_yaw = wrap_to_pi(math.atan2(-n[1], -n[0]))
    return pre_park_point, final_park_point, final_yaw


def bypass_waypoints(
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    t, n = cushion_axes(geometry)
    left_endpoint, right_endpoint = cushion_endpoints(geometry)
    left_waypoint = (
        left_endpoint[0] - config.side_clearance * t[0]
        + config.front_clearance * n[0],
        left_endpoint[1] - config.side_clearance * t[1]
        + config.front_clearance * n[1],
    )
    right_waypoint = (
        right_endpoint[0] + config.side_clearance * t[0]
        + config.front_clearance * n[0],
        right_endpoint[1] + config.side_clearance * t[1]
        + config.front_clearance * n[1],
    )
    return left_waypoint, right_waypoint


def cushion_cbf_circles(
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
) -> Tuple[CircularObstacle, ...]:
    t, _ = cushion_axes(geometry)
    radius = (
        0.5 * geometry.width
        + config.robot_safety_radius
        + config.stick_safety_extension
        + config.safety_margin
    )
    spacing = max(config.circle_spacing, 1e-3)
    count = max(2, int(math.ceil(geometry.length / spacing)) + 1)
    circles: List[CircularObstacle] = []
    for index in range(count):
        alpha = -0.5 + index / (count - 1)
        offset = alpha * geometry.length
        circles.append(
            CircularObstacle(
                geometry.center_x + offset * t[0],
                geometry.center_y + offset * t[1],
                radius,
            )
        )
    return tuple(circles)


def plan_parking_route(
    robot_position: Tuple[float, float],
    geometry: CushionGeometry,
    config: ParkingPlannerConfig,
    extra_obstacles: Sequence[CircularObstacle] = (),
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

    left_waypoint, right_waypoint = bypass_waypoints(geometry, config)
    candidates = (("left", left_waypoint), ("right", right_waypoint))
    valid_candidates = []
    for side, waypoint in candidates:
        if not _point_in_field(waypoint, config):
            continue
        if _inside_any_obstacle(waypoint, extra_obstacles):
            continue
        cost = (
            _distance(robot_position, waypoint)
            - _distance(waypoint, pre_park_point)
        )
        valid_candidates.append((cost, side, waypoint))

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

    _, side, waypoint = min(valid_candidates, key=lambda item: item[0])
    return ParkingPlan(
        False,
        side,
        (waypoint, pre_park_point, final_park_point),
        pre_park_point,
        final_park_point,
        final_yaw,
        cushion_obstacles,
        "bypass selected",
    )


def direct_segment_crosses_cushion(
    start: Tuple[float, float],
    end: Tuple[float, float],
    geometry: CushionGeometry,
) -> bool:
    t, n = cushion_axes(geometry)
    sx = start[0] - geometry.center_x
    sy = start[1] - geometry.center_y
    ex = end[0] - geometry.center_x
    ey = end[1] - geometry.center_y
    s_t = sx * t[0] + sy * t[1]
    s_n = sx * n[0] + sy * n[1]
    e_t = ex * t[0] + ey * t[1]
    e_n = ex * n[0] + ey * n[1]
    if s_n == e_n:
        return abs(s_n) <= 0.5 * geometry.width and (
            min(s_t, e_t) <= 0.5 * geometry.length
            and max(s_t, e_t) >= -0.5 * geometry.length
        )
    ratio = -s_n / (e_n - s_n)
    if ratio < 0.0 or ratio > 1.0:
        return False
    crossing_t = s_t + ratio * (e_t - s_t)
    return abs(crossing_t) <= 0.5 * geometry.length


def _point_in_field(
    point: Tuple[float, float],
    config: ParkingPlannerConfig,
) -> bool:
    x, y = point
    if config.field_min_x is not None and x < config.field_min_x:
        return False
    if config.field_max_x is not None and x > config.field_max_x:
        return False
    if config.field_min_y is not None and y < config.field_min_y:
        return False
    if config.field_max_y is not None and y > config.field_max_y:
        return False
    return True


def _inside_any_obstacle(
    point: Tuple[float, float],
    obstacles: Sequence[CircularObstacle],
) -> bool:
    for obstacle in obstacles:
        if _distance(point, (obstacle.x, obstacle.y)) <= obstacle.radius:
            return True
    return False


def _distance(
    first: Tuple[float, float],
    second: Tuple[float, float],
) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])
