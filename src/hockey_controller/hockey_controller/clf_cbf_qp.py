import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from qpsolvers import solve_qp


@dataclass(frozen=True)
class CircularObstacle:
    x: float
    y: float
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


def obstacle_arrays_valid(obstacle_x, obstacle_y, obstacle_radius):
    if len(obstacle_x) != len(obstacle_y):
        return False
    if len(obstacle_x) != len(obstacle_radius):
        return False
    return all(radius >= 0.0 for radius in obstacle_radius)


def get_u_nom(p_x, p_y, g_x, g_y, K):
    return (
        K * (g_x - p_x),
        K * (g_y - p_y),
    )


def compute_clf_values(p_x, p_y, g_x, g_y, u_x, u_y, delta, clf_gamma):
    e_x = p_x - g_x
    e_y = p_y - g_y
    value = 0.5 * (e_x**2 + e_y**2)
    residual = e_x * u_x + e_y * u_y + clf_gamma * value - delta
    return ClfValues(v=value, residual=residual)


def compute_cbf_values(p_x, p_y, u_x, u_y, obstacles, cbf_gamma):
    values = []
    for obstacle in obstacles:
        dx = p_x - obstacle.x
        dy = p_y - obstacle.y
        h = dx**2 + dy**2 - obstacle.radius**2
        residual = 2.0 * dx * u_x + 2.0 * dy * u_y + cbf_gamma * h
        values.append(CbfValue(h=h, residual=residual))
    return tuple(values)


def solve_clf_cbf_qp(
    p_x,
    p_y,
    g_x,
    g_y,
    u_nom_x,
    u_nom_y,
    obstacles,
    clf_gamma,
    cbf_gamma,
    w_delta,
    max_point_speed,
    qp_solver="cvxopt",
):
    e_x = p_x - g_x
    e_y = p_y - g_y
    value = 0.5 * (e_x**2 + e_y**2)

    constraints_a = [(e_x, e_y, -1.0)]
    constraints_b = [-clf_gamma * value]

    for obstacle in obstacles:
        dx = p_x - obstacle.x
        dy = p_y - obstacle.y
        h = dx**2 + dy**2 - obstacle.radius**2
        constraints_a.append((-2.0 * dx, -2.0 * dy, 0.0))
        constraints_b.append(cbf_gamma * h)

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

    nominal_result = _nominal_solution_if_feasible(
        p_x,
        p_y,
        g_x,
        g_y,
        u_nom_x,
        u_nom_y,
        obstacles,
        clf_gamma,
        cbf_gamma,
        w_delta,
        max_point_speed,
    )
    if nominal_result is not None:
        return nominal_result

    h_diag = (2.0, 2.0, 2.0 * w_delta)
    linear = (-2.0 * u_nom_x, -2.0 * u_nom_y, 0.0)
    P = np.diag(np.array(h_diag, dtype=float))
    q = np.array(linear, dtype=float)
    G = np.array(constraints_a, dtype=float)
    h = np.array(constraints_b, dtype=float)

    solution = solve_qp(P, q, G=G, h=h, solver=qp_solver)
    if solution is None:
        return QpResult(success=False, status="infeasible")

    values = tuple(float(value) for value in solution[:3])
    if not _all_finite(values):
        return QpResult(success=False, status="non-finite solution")

    return _build_qp_result(
        p_x,
        p_y,
        g_x,
        g_y,
        values[0],
        values[1],
        values[2],
        u_nom_x,
        u_nom_y,
        obstacles,
        clf_gamma,
        cbf_gamma,
        w_delta,
        f"qpsolvers_{qp_solver}",
    )


def _build_qp_result(
    p_x,
    p_y,
    g_x,
    g_y,
    u_x,
    u_y,
    delta,
    u_nom_x,
    u_nom_y,
    obstacles,
    clf_gamma,
    cbf_gamma,
    w_delta,
    status,
):
    clf = compute_clf_values(
        p_x,
        p_y,
        g_x,
        g_y,
        u_x,
        u_y,
        delta,
        clf_gamma,
    )
    cbfs = compute_cbf_values(
        p_x,
        p_y,
        u_x,
        u_y,
        obstacles,
        cbf_gamma,
    )
    return QpResult(
        success=True,
        status=status,
        u_x=u_x,
        u_y=u_y,
        delta=delta,
        objective=_objective((u_x, u_y, delta), u_nom_x, u_nom_y, w_delta),
        clf=clf,
        cbfs=cbfs,
    )


def _nominal_solution_if_feasible(
    p_x,
    p_y,
    g_x,
    g_y,
    u_nom_x,
    u_nom_y,
    obstacles,
    clf_gamma,
    cbf_gamma,
    w_delta,
    max_point_speed,
):
    tolerance = 1e-9
    if abs(u_nom_x) > max_point_speed + tolerance:
        return None
    if abs(u_nom_y) > max_point_speed + tolerance:
        return None

    for obstacle in obstacles:
        dx = p_x - obstacle.x
        dy = p_y - obstacle.y
        h = dx**2 + dy**2 - obstacle.radius**2
        cbf_residual = 2.0 * dx * u_nom_x + 2.0 * dy * u_nom_y + cbf_gamma * h
        if cbf_residual < -tolerance:
            return None

    e_x = p_x - g_x
    e_y = p_y - g_y
    value = 0.5 * (e_x**2 + e_y**2)
    delta = max(0.0, e_x * u_nom_x + e_y * u_nom_y + clf_gamma * value)
    return _build_qp_result(
        p_x,
        p_y,
        g_x,
        g_y,
        u_nom_x,
        u_nom_y,
        delta,
        u_nom_x,
        u_nom_y,
        obstacles,
        clf_gamma,
        cbf_gamma,
        w_delta,
        "u_nom_feasible",
    )


def _all_finite(values):
    return all(math.isfinite(value) for value in values)


def _objective(z, u_nom_x, u_nom_y, w_delta):
    return (z[0] - u_nom_x) ** 2 + (z[1] - u_nom_y) ** 2 + w_delta * z[2] ** 2
