import itertools
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


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
    qp_solver: str = "osqp",
    qp_verbose: bool = False,
) -> QpResult:
    if qp_solver == "osqp":
        return _solve_clf_cbf_qp_cvxpy_osqp(
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
            qp_verbose,
        )
    if qp_solver == "active_set":
        return _solve_clf_cbf_qp_active_set(
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
        )
    return QpResult(success=False, status=f"unsupported solver: {qp_solver}")


def _solve_clf_cbf_qp_cvxpy_osqp(
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
    qp_verbose: bool,
) -> QpResult:
    try:
        import cvxpy as cp
    except ImportError as exception:
        return QpResult(
            success=False,
            status=f"cvxpy unavailable: {exception}",
        )

    error_x = point_x - goal_x
    error_y = point_y - goal_y
    value = 0.5 * (error_x * error_x + error_y * error_y)

    z = cp.Variable(3)
    u_x = z[0]
    u_y = z[1]
    delta = z[2]

    constraints = [
        error_x * u_x + error_y * u_y
        <= -clf_gain * value + delta,
        delta >= 0.0,
        u_x >= -max_point_speed,
        u_x <= max_point_speed,
        u_y >= -max_point_speed,
        u_y <= max_point_speed,
    ]

    for obstacle in obstacles:
        dx = point_x - obstacle.x
        dy = point_y - obstacle.y
        h = dx * dx + dy * dy - obstacle.radius * obstacle.radius
        constraints.append(
            2.0 * dx * u_x + 2.0 * dy * u_y >= -cbf_gain * h
        )

    objective = cp.Minimize(
        cp.square(u_x - u_nom_x)
        + cp.square(u_y - u_nom_y)
        + slack_weight * cp.square(delta)
    )
    problem = cp.Problem(objective, constraints)

    try:
        problem.solve(solver=cp.OSQP, warm_start=True, verbose=qp_verbose)
    except Exception as exception:
        return QpResult(success=False, status=f"osqp exception: {exception}")

    if problem.status not in ("optimal", "optimal_inaccurate"):
        return QpResult(success=False, status=str(problem.status))
    if z.value is None:
        return QpResult(success=False, status="no solution")

    u_x_value = float(z.value[0])
    u_y_value = float(z.value[1])
    delta_value = float(z.value[2])
    if not _all_finite((u_x_value, u_y_value, delta_value)):
        return QpResult(success=False, status="non-finite solution")

    return _build_qp_result(
        point_x,
        point_y,
        goal_x,
        goal_y,
        u_x_value,
        u_y_value,
        delta_value,
        u_nom_x,
        u_nom_y,
        obstacles,
        clf_gain,
        cbf_gain,
        slack_weight,
        str(problem.status),
    )


def _solve_clf_cbf_qp_active_set(
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
) -> QpResult:
    constraints_a: List[Tuple[float, float, float]] = []
    constraints_b: List[float] = []

    error_x = point_x - goal_x
    error_y = point_y - goal_y
    value = 0.5 * (error_x * error_x + error_y * error_y)

    constraints_a.append((error_x, error_y, -1.0))
    constraints_b.append(-clf_gain * value)

    for obstacle in obstacles:
        dx = point_x - obstacle.x
        dy = point_y - obstacle.y
        h = dx * dx + dy * dy - obstacle.radius * obstacle.radius
        constraints_a.append((-2.0 * dx, -2.0 * dy, 0.0))
        constraints_b.append(cbf_gain * h)

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

    h_diag = (2.0, 2.0, 2.0 * slack_weight)
    linear = (-2.0 * u_nom_x, -2.0 * u_nom_y, 0.0)

    best_z = None
    best_objective = math.inf
    constraint_indices = range(len(constraints_a))

    for active_count in range(0, 4):
        for active_set in itertools.combinations(constraint_indices, active_count):
            candidate = _solve_kkt(
                h_diag,
                linear,
                constraints_a,
                constraints_b,
                active_set,
            )
            if candidate is None:
                continue
            if not _all_finite(candidate):
                continue
            if not _is_feasible(candidate, constraints_a, constraints_b):
                continue

            objective = _objective(candidate, u_nom_x, u_nom_y, slack_weight)
            if objective < best_objective:
                best_objective = objective
                best_z = candidate

    if best_z is None:
        return QpResult(success=False, status="infeasible")

    u_x, u_y, delta = best_z
    return _build_qp_result(
        point_x,
        point_y,
        goal_x,
        goal_y,
        u_x,
        u_y,
        delta,
        u_nom_x,
        u_nom_y,
        obstacles,
        clf_gain,
        cbf_gain,
        slack_weight,
        "optimal",
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
    return QpResult(
        success=True,
        status=status,
        u_x=u_x,
        u_y=u_y,
        delta=delta,
        objective=_objective((u_x, u_y, delta), u_nom_x, u_nom_y, slack_weight),
        clf=clf,
        cbfs=cbfs,
    )


def _solve_kkt(
    h_diag: Tuple[float, float, float],
    linear: Tuple[float, float, float],
    constraints_a: Sequence[Tuple[float, float, float]],
    constraints_b: Sequence[float],
    active_set: Sequence[int],
) -> Optional[Tuple[float, float, float]]:
    size = 3 + len(active_set)
    matrix = [[0.0 for _ in range(size)] for _ in range(size)]
    rhs = [0.0 for _ in range(size)]

    for i in range(3):
        matrix[i][i] = h_diag[i]
        rhs[i] = -linear[i]

    for column, constraint_index in enumerate(active_set, start=3):
        row = constraints_a[constraint_index]
        for i in range(3):
            matrix[i][column] = row[i]
            matrix[column][i] = row[i]
        rhs[column] = constraints_b[constraint_index]

    solution = _solve_linear_system(matrix, rhs)
    if solution is None:
        return None
    return (solution[0], solution[1], solution[2])


def _solve_linear_system(
    matrix: List[List[float]],
    rhs: List[float],
) -> Optional[List[float]]:
    size = len(rhs)
    a = [row[:] + [rhs_value] for row, rhs_value in zip(matrix, rhs)]

    for pivot_col in range(size):
        pivot_row = max(
            range(pivot_col, size),
            key=lambda row_index: abs(a[row_index][pivot_col]),
        )
        if abs(a[pivot_row][pivot_col]) < 1e-10:
            return None
        if pivot_row != pivot_col:
            a[pivot_col], a[pivot_row] = a[pivot_row], a[pivot_col]

        pivot = a[pivot_col][pivot_col]
        for col in range(pivot_col, size + 1):
            a[pivot_col][col] /= pivot

        for row in range(size):
            if row == pivot_col:
                continue
            factor = a[row][pivot_col]
            if abs(factor) < 1e-14:
                continue
            for col in range(pivot_col, size + 1):
                a[row][col] -= factor * a[pivot_col][col]

    return [a[row][size] for row in range(size)]


def _is_feasible(
    z: Tuple[float, float, float],
    constraints_a: Sequence[Tuple[float, float, float]],
    constraints_b: Sequence[float],
) -> bool:
    tolerance = 1e-7
    for row, bound in zip(constraints_a, constraints_b):
        value = row[0] * z[0] + row[1] * z[1] + row[2] * z[2]
        if value > bound + tolerance:
            return False
    return True


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
