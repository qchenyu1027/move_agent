"""不依赖 ROS 的抓取航路点规划与安全检查。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


class PlanError(ValueError):
    """航路点不满足安全约束。"""


@dataclass(frozen=True)
class Waypoint:
    """base_link 坐标系下的工具目标。"""

    name: str
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _tuple3(values: Sequence[float], label: str) -> tuple[float, float, float]:
    if len(values) != 3 or not _finite(values):
        raise PlanError(f'{label} 必须是 3 个有限数值')
    return tuple(float(value) for value in values)


def _tuple4(values: Sequence[float], label: str) -> tuple[float, float, float, float]:
    if len(values) != 4 or not _finite(values):
        raise PlanError(f'{label} 必须是 4 个有限数值')
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm < 1e-12:
        raise PlanError(f'{label} 不能是零四元数')
    return tuple(float(value) / norm for value in values)


def distance(first: Sequence[float], second: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(first, second)))


def held_joint_configuration(
    hold: Sequence[float], feedback: Sequence[float], side: str,
) -> list[float]:
    """固定躯干、头部和另一侧手臂，只更新所选手臂的实测关节角。"""
    if len(hold) != 22 or len(feedback) != 22 or not _finite(hold) or not _finite(feedback):
        raise PlanError('关节目标和反馈必须各包含 22 个有限数值')
    if side not in ('left', 'right'):
        raise PlanError('抓取侧必须为 left 或 right')
    result = [float(value) for value in hold]
    start = 8 if side == 'left' else 15
    result[start:start + 7] = [float(value) for value in feedback[start:start + 7]]
    return result


def interpolate_segment(first: Sequence[float], second: Sequence[float], max_step_m: float):
    """以不超过 max_step_m 的间隔生成直线离散点。"""
    if not math.isfinite(max_step_m) or max_step_m <= 0.0:
        raise PlanError('max_step_m 必须为正数')
    start = _tuple3(first, '起点')
    end = _tuple3(second, '终点')
    count = max(1, math.ceil(distance(start, end) / max_step_m))
    for index in range(1, count + 1):
        ratio = index / count
        yield tuple(a + (b - a) * ratio for a, b in zip(start, end))


def _deduplicate(points: list[Waypoint]) -> list[Waypoint]:
    result: list[Waypoint] = []
    for point in points:
        if (result and point.name != 'pregrasp' and
                distance(result[-1].position, point.position) < 1e-9):
            continue
        result.append(point)
    return result


def densify_waypoints(
    waypoints: Sequence[Waypoint], max_step_m: float,
    *, direct_targets: Sequence[str] = (),
) -> list[Waypoint]:
    """接近阶段展开为小步；指定的最终目标不插入等待点。"""
    if not waypoints:
        raise PlanError('航路点不能为空')
    if not math.isfinite(max_step_m) or max_step_m <= 0.0:
        raise PlanError('max_step_m 必须为正数')
    result = [waypoints[0]]
    for first, second in zip(waypoints, waypoints[1:]):
        if second.name in direct_targets:
            result.append(second)
            continue
        count = max(1, math.ceil(distance(first.position, second.position) / max_step_m))
        for index in range(1, count + 1):
            ratio = index / count
            position = tuple(
                a + (b - a) * ratio
                for a, b in zip(first.position, second.position))
            name = second.name if index == count else f'{second.name}[{index}/{count}]'
            result.append(Waypoint(name, position, second.quaternion))
    return result


def validate_waypoints(
    waypoints: Sequence[Waypoint],
    *,
    table_z_m: float,
    table_clearance_m: float,
    start_clearance_m: float = 0.02,
    max_step_m: float,
) -> None:
    """检查航路点和离散线段的桌面安全高度。

    当前工具点允许低于航行安全高度，但第一段只能垂直抬高；它仍须高于
    桌面加 start_clearance_m，避免把已经存在的起始姿态误判为规划错误。
    """
    if not math.isfinite(table_z_m) or not math.isfinite(table_clearance_m) or not math.isfinite(start_clearance_m):
        raise PlanError('桌面高度和安全余量必须是有限数值')
    if table_clearance_m <= 0.0 or start_clearance_m <= 0.0:
        raise PlanError('桌面和起始安全余量必须大于 0')
    if not waypoints:
        raise PlanError('航路点不能为空')
    minimum_z = table_z_m + table_clearance_m
    start_minimum_z = table_z_m + start_clearance_m
    if waypoints[0].position[2] < start_minimum_z:
        raise PlanError(
            f'起始工具点的 Z={waypoints[0].position[2]:.3f} m 低于起始安全高度 '
            f'{start_minimum_z:.3f} m')
    for point in waypoints[1:]:
        if point.position[2] < minimum_z:
            raise PlanError(
                f'航路点 {point.name} 的 Z={point.position[2]:.3f} m 低于桌面安全高度 '
                f'{minimum_z:.3f} m')
    for segment_index, (first, second) in enumerate(zip(waypoints, waypoints[1:])):
        segment_minimum_z = start_minimum_z if segment_index == 0 else minimum_z
        for position in interpolate_segment(first.position, second.position, max_step_m):
            if position[2] < segment_minimum_z:
                raise PlanError(
                    f'线段 {first.name}->{second.name} 经过起始安全高度以下：'
                    f'Z={position[2]:.3f} m')


def build_grasp_plan(
    current_position: Sequence[float],
    current_quaternion: Sequence[float],
    object_position: Sequence[float],
    *,
    grasp_offset_mm: Sequence[float] = (0.0, 0.0, -40.0),
    approach_front_mm: float = 80.0,
    approach_height_mm: float = 30.0,
    overhead_mm: float = 80.0,
    lift_height_mm: float = 150.0,
    table_z_m: float,
    table_clearance_mm: float = 100.0,
    start_clearance_mm: float = 20.0,
    max_step_mm: float = 10.0,
) -> list[Waypoint]:
    """生成抬高、平移、预抓取点闭合、垂直抬升的航路。"""
    current = _tuple3(current_position, '当前工具位置')
    current_q = _tuple4(current_quaternion, '当前工具姿态')
    object_xyz = _tuple3(object_position, '物体位置')
    offset = _tuple3(grasp_offset_mm, '抓取偏移')
    values = (approach_front_mm, approach_height_mm, overhead_mm,
              lift_height_mm, table_clearance_mm, start_clearance_mm, max_step_mm)
    if not _finite(values) or any(value < 0.0 for value in values):
        raise PlanError('路径距离和安全余量必须是有限非负数')
    if overhead_mm <= 0.0 or lift_height_mm <= 0.0 or max_step_mm <= 0.0:
        raise PlanError('overhead_mm、lift_height_mm、max_step_mm 必须大于 0')

    grasp_reference = tuple(a + b / 1000.0 for a, b in zip(object_xyz, offset))
    pregrasp = (
        grasp_reference[0] - approach_front_mm / 1000.0,
        grasp_reference[1],
        grasp_reference[2] + approach_height_mm / 1000.0,
    )
    minimum_z = table_z_m + table_clearance_mm / 1000.0
    safe_z = max(grasp_reference[2] + overhead_mm / 1000.0, minimum_z)
    travel_z = max(current[2], safe_z)
    current_safe = (current[0], current[1], travel_z)
    above_pregrasp = (pregrasp[0], pregrasp[1], travel_z)
    lift = (pregrasp[0], pregrasp[1], pregrasp[2] + lift_height_mm / 1000.0)

    waypoints = _deduplicate([
        Waypoint('current', current, current_q),
        Waypoint('lift_before_translation', current_safe, current_q),
        Waypoint('above_pregrasp', above_pregrasp, current_q),
        Waypoint('pregrasp', pregrasp, current_q),
        Waypoint('lift_after_grasp', lift, current_q),
    ])
    validate_waypoints(
        waypoints,
        table_z_m=table_z_m,
        table_clearance_m=table_clearance_mm / 1000.0,
        start_clearance_m=start_clearance_mm / 1000.0,
        max_step_m=max_step_mm / 1000.0,
    )
    return waypoints
