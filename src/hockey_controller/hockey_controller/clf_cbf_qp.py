import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class CircularObstacle:
    x: float
    y: float
    radius: float


@dataclass(frozen=True)
class DynamicCircularObstacle:
    x: float
    y: float
    velocity_x: float
    velocity_y: float
    radius: float


@dataclass(frozen=True)
class ClfValues:
    v: float
    residual: float


@dataclass(frozen=True)
class CbfValue:
    h: float
    residual: float


@dataclass(frozen=True)
class QpResult:
    success: bool
    status: str
    u_x: float = 0.0
    u_y: float = 0.0
    delta: float = 0.0
    objective: float = math.inf
    clf: Optional[ClfValues] = None
    cbfs: Tuple[CbfValue, ...] = ()
    dynamic_cbfs: Tuple[CbfValue, ...] = ()


def obstacle_arrays_valid(
    obstacle_x: Sequence[float],
    obstacle_y: Sequence[float],
    obstacle_radius: Sequence[float],
) -> bool:
    if len(obstacle_x) != len(obstacle_y):
        return False
    if len(obstacle_x) != len(obstacle_radius):
        return False
    return all(radius >= 0.0 for radius in obstacle_radius)


def compute_nominal_point_velocity(
    point_x: float,
    point_y: float,
    goal_x: float,
    goal_y: float,
    point_gain: float,
) -> Tuple[float, float]:
    return (
        point_gain * (goal_x - point_x),
        point_gain * (goal_y - point_y),
    )


def compute_clf_values(
    point_x: float,
    point_y: float,
    goal_x: float,
    goal_y: float,
    u_x: float,
    u_y: float,
    delta: float,
    clf_gain: float,
) -> ClfValues:
    error_x = point_x - goal_x
    error_y = point_y - goal_y
    value = 0.5 * (error_x * error_x + error_y * error_y)
    residual = (
        error_x * u_x
        + error_y * u_y
        + clf_gain * value
        - delta
    )
    return ClfValues(v=value, residual=residual)


def compute_cbf_values(
    point_x: float,
    point_y: float,
    u_x: float,
    u_y: float,
    obstacles: Sequence[CircularObstacle],
    cbf_gain: float,
) -> Tuple[CbfValue, ...]:
    values = []
    for obstacle in obstacles:
        dx = point_x - obstacle.x
        dy = point_y - obstacle.y
        h = dx * dx + dy * dy - obstacle.radius * obstacle.radius
        residual = 2.0 * dx * u_x + 2.0 * dy * u_y + cbf_gain * h
        values.append(CbfValue(h=h, residual=residual))
    return tuple(values)


def compute_dynamic_cbf_values(
    point_x: float,
    point_y: float,
    u_x: float,
    u_y: float,
    obstacles: Sequence[DynamicCircularObstacle],
    cbf_gain: float,
) -> Tuple[CbfValue, ...]:
    values = []
    for obstacle in obstacles:
        dx = point_x - obstacle.x
        dy = point_y - obstacle.y
        h = dx * dx + dy * dy - obstacle.radius * obstacle.radius
        residual = (
            2.0 * dx * (u_x - obstacle.velocity_x)
            + 2.0 * dy * (u_y - obstacle.velocity_y)
            + cbf_gain * h
        )
        values.append(CbfValue(h=h, residual=residual))
    return tuple(values)


def solve_clf_cbf_qp(
    point_x: float,
    point_y: float,
    goal_x: float,
    goal_y: float,
    u_nom_x: float,
    u_nom_y: float,
    obstacles: Sequence[CircularObstacle],
    clf_gain: float,
    cbf_gain: float,
    slack_weight: float,
    max_point_speed: float,
    qp_solver: str = "cvxopt",
    qp_verbose: bool = False,
    dynamic_obstacles: Sequence[DynamicCircularObstacle] = (),
    dynamic_cbf_gain: Optional[float] = None,
) -> QpResult:
    dynamic_gain = cbf_gain if dynamic_cbf_gain is None else dynamic_cbf_gain
    return _solve_clf_cbf_qp_qpsolvers(
        point_x,
        point_y,
        goal_x,
        goal_y,
        u_nom_x,
        u_nom_y,
        obstacles,
        clf_gain,
        cbf_gain,
        slack_weight,
        max_point_speed,
        qp_solver,
        qp_verbose,
        dynamic_obstacles,
        dynamic_gain,
    )


def _solve_clf_cbf_qp_qpsolvers(
    point_x: float,
    point_y: float,
    goal_x: float,
    goal_y: float,
    u_nom_x: float,
    u_nom_y: float,
    obstacles: Sequence[CircularObstacle],
    clf_gain: float,
    cbf_gain: float,
    slack_weight: float,
    max_point_speed: float,
    qp_solver: str,
    qp_verbose: bool,
    dynamic_obstacles: Sequence[DynamicCircularObstacle],
    dynamic_cbf_gain: float,
) -> QpResult:
    error_x = point_x - goal_x
    error_y = point_y - goal_y
    value = 0.5 * (error_x * error_x + error_y * error_y)

    constraints_a = [(error_x, error_y, -1.0)]
    constraints_b = [-clf_gain * value]

    for obstacle in obstacles:
        dx = point_x - obstacle.x
        dy = point_y - obstacle.y
        h = dx * dx + dy * dy - obstacle.radius * obstacle.radius
        constraints_a.append((-2.0 * dx, -2.0 * dy, 0.0))
        constraints_b.append(cbf_gain * h)

    for obstacle in dynamic_obstacles:
        dx = point_x - obstacle.x
        dy = point_y - obstacle.y
        h = dx * dx + dy * dy - obstacle.radius * obstacle.radius
        constraints_a.append((-2.0 * dx, -2.0 * dy, 0.0))
        constraints_b.append(
            dynamic_cbf_gain * h
            - 2.0 * dx * obstacle.velocity_x
            - 2.0 * dy * obstacle.velocity_y
        )

    constraints_a.extend(
        [
            (-1.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, -1.0),
        ]
    )
    constraints_b.extend(
        [
            max_point_speed,
            max_point_speed,
            max_point_speed,
            max_point_speed,
            0.0,
        ]
    )

    solution = _solve_qp_with_qpsolvers(
        (2.0, 2.0, 2.0 * slack_weight),
        (-2.0 * u_nom_x, -2.0 * u_nom_y, 0.0),
        constraints_a,
        constraints_b,
        qp_solver,
        qp_verbose,
    )
    if not solution.success:
        return solution

    return _build_qp_result(
        point_x,
        point_y,
        goal_x,
        goal_y,
        solution.u_x,
        solution.u_y,
        solution.delta,
        u_nom_x,
        u_nom_y,
        obstacles,
        clf_gain,
        cbf_gain,
        slack_weight,
        solution.status,
        dynamic_obstacles,
        dynamic_cbf_gain,
    )


def _build_qp_result(
    point_x: float,
    point_y: float,
    goal_x: float,
    goal_y: float,
    u_x: float,
    u_y: float,
    delta: float,
    u_nom_x: float,
    u_nom_y: float,
    obstacles: Sequence[CircularObstacle],
    clf_gain: float,
    cbf_gain: float,
    slack_weight: float,
    status: str,
    dynamic_obstacles: Sequence[DynamicCircularObstacle] = (),
    dynamic_cbf_gain: Optional[float] = None,
) -> QpResult:
    clf = compute_clf_values(
        point_x,
        point_y,
        goal_x,
        goal_y,
        u_x,
        u_y,
        delta,
        clf_gain,
    )
    cbfs = compute_cbf_values(
        point_x,
        point_y,
        u_x,
        u_y,
        obstacles,
        cbf_gain,
    )
    dynamic_gain = cbf_gain if dynamic_cbf_gain is None else dynamic_cbf_gain
    dynamic_cbfs = compute_dynamic_cbf_values(
        point_x,
        point_y,
        u_x,
        u_y,
        dynamic_obstacles,
        dynamic_gain,
    )
    return QpResult(
        success=True,
        status=status,
        u_x=u_x,
        u_y=u_y,
        delta=delta,
        objective=_objective((u_x, u_y, delta), u_nom_x, u_nom_y, slack_weight),
        clf=clf,
        cbfs=cbfs,
        dynamic_cbfs=dynamic_cbfs,
    )


def _solve_qp_with_qpsolvers(
    h_diag: Tuple[float, float, float],
    linear: Tuple[float, float, float],
    constraints_a: Sequence[Tuple[float, float, float]],
    constraints_b: Sequence[float],
    solver: str,
    verbose: bool,
) -> QpResult:
    del verbose
    try:
        import numpy as np
        from qpsolvers import solve_qp
    except ImportError as exception:
        return QpResult(
            success=False,
            status=f"qpsolvers unavailable: {exception}",
        )

    p_matrix = np.diag(np.array(h_diag, dtype=float))
    q_vector = np.array(linear, dtype=float)
    g_matrix = np.array(constraints_a, dtype=float)
    h_vector = np.array(constraints_b, dtype=float)
    try:
        solution = solve_qp(
            p_matrix,
            q_vector,
            G=g_matrix,
            h=h_vector,
            solver=solver,
        )
    except Exception as exception:
        return QpResult(
            success=False,
            status=f"qpsolvers {solver} exception: {exception}",
        )

    if solution is None:
        return QpResult(success=False, status=f"qpsolvers {solver}: no solution")

    values = tuple(float(value) for value in solution[:3])
    if not _all_finite(values):
        return QpResult(success=False, status="non-finite solution")
    return QpResult(
        success=True,
        status=f"qpsolvers_{solver}",
        u_x=values[0],
        u_y=values[1],
        delta=values[2],
        objective=_quadratic_objective(values, h_diag, linear),
    )


def _all_finite(values: Sequence[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _objective(
    z: Tuple[float, float, float],
    u_nom_x: float,
    u_nom_y: float,
    slack_weight: float,
) -> float:
    return (
        (z[0] - u_nom_x) * (z[0] - u_nom_x)
        + (z[1] - u_nom_y) * (z[1] - u_nom_y)
        + slack_weight * z[2] * z[2]
    )


def _quadratic_objective(
    z: Tuple[float, float, float],
    h_diag: Tuple[float, float, float],
    linear: Tuple[float, float, float],
) -> float:
    return 0.5 * (
        h_diag[0] * z[0] * z[0]
        + h_diag[1] * z[1] * z[1]
        + h_diag[2] * z[2] * z[2]
    ) + linear[0] * z[0] + linear[1] * z[1] + linear[2] * z[2]
