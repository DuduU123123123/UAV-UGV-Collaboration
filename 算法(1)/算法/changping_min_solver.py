#!/usr/bin/env python3
"""昌平线车—机协同巡检最小实例基准求解器。

算法采用事件驱动的时空状态网络与标签设置法。状态为：
    (当前时刻, 移动机巢所在站, 无人机SOC, 已完成任务集合, 已换电次数)

仅使用 Python 标准库，适合先验证状态转移、时间窗、能量和会合逻辑。
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
from dataclasses import dataclass
from itertools import count
from pathlib import Path
from time import perf_counter
from typing import Any, Iterable


@dataclass(frozen=True, slots=True)
class State:
    time_min: int
    station_index: int
    soc_pct: int
    completed_mask: int
    swaps_used: int


@dataclass(slots=True)
class Context:
    data: dict[str, Any]
    stations: list[dict[str, Any]]
    tasks: list[dict[str, Any]]
    task_locations: dict[str, dict[str, Any]]
    station_index: dict[str, int]
    segment_distance: list[int]
    segment_travel_time: list[int]
    start_index: int
    end_index: int
    horizon: int
    time_step: int
    soc_step: int
    reserve_soc: int
    initial_soc: int
    uav_id: str
    spare_batteries: int
    nest_speed_kmh: float
    uav_speed_mps: float
    usable_endurance_min: float
    swap_time_min: int
    allow_reverse: bool


def ceil_to_step(value: float, step: int) -> int:
    if value <= 0:
        return 0
    return int(math.ceil((value - 1e-12) / step) * step)


def load_instance(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    # 轻量场景文件只保存任务子集，避免复制整份基础实例。
    if "base_instance" in data:
        base_path = (path.parent / data["base_instance"]).resolve()
        base_data = load_instance(base_path)
        selected_task_ids = set(data["task_ids"])
        unknown = selected_task_ids.difference(task["task_id"] for task in base_data["tasks"])
        if unknown:
            raise ValueError(f"场景引用了不存在的任务: {sorted(unknown)}")
        base_data["tasks"] = [task for task in base_data["tasks"] if task["task_id"] in selected_task_ids]
        selected_locations = {task["location_id"] for task in base_data["tasks"]}
        base_data["task_locations"] = [
            location for location in base_data["task_locations"] if location["location_id"] in selected_locations
        ]
        base_data.setdefault("metadata", {})["scenario_id"] = data.get("scenario_id", path.stem)
        base_data["metadata"]["scenario_notes"] = data.get("notes", "")
        return base_data
    return data


def apply_time_horizon_override(data: dict[str, Any], new_horizon: int) -> None:
    """统一修改总时域、车辆班次结束时间及原本贴近总时域的任务截止时间。"""
    parameters = data["parameters"]
    old_horizon = int(parameters["time_horizon_min"])
    time_step = int(parameters["time_step_min"])
    if new_horizon <= 0 or new_horizon % time_step:
        raise ValueError("覆盖时间范围必须为正且能被时间步长整除")

    parameters["time_horizon_min"] = new_horizon
    for nest in data["resources"]["mobile_nests"]:
        if int(nest["shift_end_min"]) == old_horizon:
            nest["shift_end_min"] = new_horizon
    for task in data["tasks"]:
        if int(task["latest_finish_min"]) == old_horizon:
            task["latest_finish_min"] = new_horizon


def validate_instance(data: dict[str, Any]) -> None:
    required = {
        "parameters",
        "stations",
        "segments",
        "directed_vehicle_arcs",
        "task_locations",
        "tasks",
        "resources",
    }
    missing = required.difference(data)
    if missing:
        raise ValueError(f"缺少顶层字段: {sorted(missing)}")

    stations = sorted(data["stations"], key=lambda row: row["order"])
    if len(stations) < 2:
        raise ValueError("至少需要两个车辆站点")
    if len({row["location_id"] for row in stations}) != len(stations):
        raise ValueError("站点location_id存在重复")
    if any(not row["vehicle_accessible"] for row in stations):
        raise ValueError("stations中存在车辆不可达节点")
    if any(stations[i]["chainage_m"] >= stations[i + 1]["chainage_m"] for i in range(len(stations) - 1)):
        raise ValueError("站点chainage_m必须严格递增")

    segments = data["segments"]
    if len(segments) != len(stations) - 1:
        raise ValueError("首版算法要求线路为连续链，区间数应等于站点数减一")
    for index, segment in enumerate(segments):
        left, right = stations[index], stations[index + 1]
        expected_distance = right["chainage_m"] - left["chainage_m"]
        if segment["from_location_id"] != left["location_id"] or segment["to_location_id"] != right["location_id"]:
            raise ValueError(f"区间{segment['segment_id']}与站点顺序不一致")
        if segment["distance_m"] != expected_distance:
            raise ValueError(f"区间{segment['segment_id']}距离与累计里程不一致")

    directed = data["directed_vehicle_arcs"]
    arc_keys = {(row["from_location_id"], row["to_location_id"], row["distance_m"]) for row in directed}
    for segment in segments:
        forward = (segment["from_location_id"], segment["to_location_id"], segment["distance_m"])
        reverse = (segment["to_location_id"], segment["from_location_id"], segment["distance_m"])
        if forward not in arc_keys or reverse not in arc_keys:
            raise ValueError(f"区间{segment['segment_id']}缺少正向或反向车辆弧")

    parameters = data["parameters"]
    horizon = int(parameters["time_horizon_min"])
    time_step = int(parameters["time_step_min"])
    if horizon <= 0 or time_step <= 0 or horizon % time_step:
        raise ValueError("时间范围必须为正且能被时间步长整除")
    if not parameters.get("one_task_per_sortie", True):
        raise ValueError("首版算法仅支持每架次一个任务")

    task_location_ids = {row["location_id"] for row in data["task_locations"]}
    task_ids: set[str] = set()
    for task in data["tasks"]:
        if task["task_id"] in task_ids:
            raise ValueError(f"任务ID重复: {task['task_id']}")
        task_ids.add(task["task_id"])
        if task["location_id"] not in task_location_ids:
            raise ValueError(f"任务{task['task_id']}引用了不存在的任务点")
        if task["service_time_min"] <= 0 or task["service_time_min"] % time_step:
            raise ValueError(f"任务{task['task_id']}服务时间未与时间步长对齐")
        if not (0 <= task["earliest_start_min"] <= task["latest_finish_min"] <= horizon):
            raise ValueError(f"任务{task['task_id']}时间窗无效")

    nests = data["resources"].get("mobile_nests", [])
    uavs = data["resources"].get("uavs", [])
    if len(nests) != 1 or len(uavs) != 1:
        raise ValueError("首版算法仅支持1辆移动机巢和1架无人机")


def build_context(
    data: dict[str, Any],
    *,
    spare_batteries_override: int | None = None,
    allow_reverse: bool = False,
) -> Context:
    validate_instance(data)
    stations = sorted(data["stations"], key=lambda row: row["order"])
    station_index = {row["location_id"]: idx for idx, row in enumerate(stations)}
    tasks = list(data["tasks"])
    task_locations = {row["location_id"]: row for row in data["task_locations"]}
    parameters = data["parameters"]
    nest = data["resources"]["mobile_nests"][0]
    uav = data["resources"]["uavs"][0]
    time_step = int(parameters["time_step_min"])
    nest_speed_kmh = float(nest["speed_kmh"])

    segment_distance = [int(row["distance_m"]) for row in data["segments"]]
    segment_travel_time = [
        ceil_to_step(distance / 1000.0 / nest_speed_kmh * 60.0, time_step)
        for distance in segment_distance
    ]
    soc_levels = sorted(int(value) for value in parameters["soc_levels_pct"])
    if len(soc_levels) < 2:
        raise ValueError("soc_levels_pct至少需要两个水平")
    soc_step = min(b - a for a, b in zip(soc_levels, soc_levels[1:]))
    spare_batteries = int(nest["spare_battery_sets"])
    if spare_batteries_override is not None:
        spare_batteries = spare_batteries_override

    return Context(
        data=data,
        stations=stations,
        tasks=tasks,
        task_locations=task_locations,
        station_index=station_index,
        segment_distance=segment_distance,
        segment_travel_time=segment_travel_time,
        start_index=station_index[nest["start_location_id"]],
        end_index=station_index[nest["end_location_id"]],
        horizon=int(parameters["time_horizon_min"]),
        time_step=time_step,
        soc_step=soc_step,
        reserve_soc=int(uav["minimum_reserve_soc_pct"]),
        initial_soc=int(uav["initial_soc_pct"]),
        uav_id=str(uav["uav_id"]),
        spare_batteries=spare_batteries,
        nest_speed_kmh=nest_speed_kmh,
        uav_speed_mps=float(uav["operational_speed_mps"]),
        usable_endurance_min=float(uav["usable_endurance_min"]),
        swap_time_min=int(uav["battery_swap_time_min"]),
        allow_reverse=allow_reverse,
    )


def path_segment_indices(start: int, end: int) -> Iterable[int]:
    if start < end:
        return range(start, end)
    return range(end, start)


def vehicle_path_time(ctx: Context, start: int, end: int) -> int:
    return sum(ctx.segment_travel_time[idx] for idx in path_segment_indices(start, end))


def vehicle_path_distance(ctx: Context, start: int, end: int) -> int:
    return sum(ctx.segment_distance[idx] for idx in path_segment_indices(start, end))


def soc_drop_for_duration(ctx: Context, active_min: int) -> int:
    usable_soc = 100 - ctx.reserve_soc
    raw_drop = active_min / ctx.usable_endurance_min * usable_soc
    return ceil_to_step(raw_drop, ctx.soc_step)


def state_core(state: State) -> tuple[int, int, int, int]:
    return state.station_index, state.soc_pct, state.completed_mask, state.swaps_used


def generate_transitions(ctx: Context, state: State) -> Iterable[tuple[State, dict[str, Any], int, int]]:
    station = ctx.stations[state.station_index]

    # 无人机在车上时，移动机巢可以沿线路移动。默认只向终点方向移动。
    neighbours: list[int] = []
    if state.station_index < len(ctx.stations) - 1:
        neighbours.append(state.station_index + 1)
    if ctx.allow_reverse and state.station_index > 0:
        neighbours.append(state.station_index - 1)
    for next_index in neighbours:
        duration = vehicle_path_time(ctx, state.station_index, next_index)
        end_time = state.time_min + duration
        if end_time <= ctx.horizon:
            next_station = ctx.stations[next_index]
            next_state = State(end_time, next_index, state.soc_pct, state.completed_mask, state.swaps_used)
            action = {
                "type": "vehicle_move_with_uav_onboard",
                "uav_id": ctx.uav_id,
                "start_min": state.time_min,
                "end_min": end_time,
                "nest_from": station["location_id"],
                "nest_to": next_station["location_id"],
                "vehicle_distance_m": vehicle_path_distance(ctx, state.station_index, next_index),
                "soc_before_pct": state.soc_pct,
                "soc_after_pct": state.soc_pct,
                "current_soc_pct": state.soc_pct,
                "swaps_used_before": state.swaps_used,
                "swaps_used_after": state.swaps_used,
            }
            yield next_state, action, 0, duration

    # 换电弧：只记录换电次数，不维护逐时刻库存变量。
    if state.soc_pct < ctx.initial_soc and state.swaps_used < ctx.spare_batteries:
        end_time = state.time_min + ctx.swap_time_min
        if end_time <= ctx.horizon:
            next_state = State(end_time, state.station_index, ctx.initial_soc, state.completed_mask, state.swaps_used + 1)
            action = {
                "type": "battery_swap",
                "uav_id": ctx.uav_id,
                "start_min": state.time_min,
                "end_min": end_time,
                "nest_from": station["location_id"],
                "nest_to": station["location_id"],
                "soc_before_pct": state.soc_pct,
                "soc_after_pct": ctx.initial_soc,
                "current_soc_pct": ctx.initial_soc,
                "swaps_used_before": state.swaps_used,
                "swaps_used_after": state.swaps_used + 1,
            }
            yield next_state, action, 0, 0

    # 联合服务弧：无人机执行一个任务，机巢可同时驶向回收站。
    if ctx.allow_reverse:
        recovery_indices = range(len(ctx.stations))
    else:
        recovery_indices = range(state.station_index, len(ctx.stations))

    launch_chainage = float(station["chainage_m"])
    for task_index, task in enumerate(ctx.tasks):
        task_bit = 1 << task_index
        if state.completed_mask & task_bit:
            continue
        task_location = ctx.task_locations[task["location_id"]]
        task_chainage = float(task_location["chainage_m"])
        outbound_distance = abs(task_chainage - launch_chainage)
        outbound_min = ceil_to_step(outbound_distance / ctx.uav_speed_mps / 60.0, ctx.time_step)
        earliest_service = ceil_to_step(float(task["earliest_start_min"]), ctx.time_step)
        service_start = max(state.time_min + outbound_min, earliest_service)
        service_end = service_start + int(task["service_time_min"])
        if service_end > int(task["latest_finish_min"]):
            continue

        for recovery_index in recovery_indices:
            recovery_station = ctx.stations[recovery_index]
            recovery_chainage = float(recovery_station["chainage_m"])
            return_distance = abs(task_chainage - recovery_chainage)
            return_min = ceil_to_step(return_distance / ctx.uav_speed_mps / 60.0, ctx.time_step)
            drone_finish = service_end + return_min
            drone_duration = drone_finish - state.time_min
            nest_duration = vehicle_path_time(ctx, state.station_index, recovery_index)
            joint_duration = max(drone_duration, nest_duration)
            end_time = state.time_min + joint_duration
            if end_time > ctx.horizon:
                continue

            energy_drop = soc_drop_for_duration(ctx, joint_duration)
            next_soc = state.soc_pct - energy_drop
            if next_soc < ctx.reserve_soc:
                continue

            next_state = State(
                end_time,
                recovery_index,
                next_soc,
                state.completed_mask | task_bit,
                state.swaps_used,
            )
            action = {
                "type": "joint_sortie_service",
                "uav_id": ctx.uav_id,
                "task_id": task["task_id"],
                "task_type": task["task_type"],
                "start_min": state.time_min,
                "end_min": end_time,
                "service_start_min": service_start,
                "service_end_min": service_end,
                "nest_from": station["location_id"],
                "nest_to": recovery_station["location_id"],
                "uav_launch": station["location_id"],
                "uav_recovery": recovery_station["location_id"],
                "flight_distance_m": round(outbound_distance + return_distance, 3),
                "drone_active_min": joint_duration,
                "vehicle_travel_min": nest_duration,
                "soc_before_pct": state.soc_pct,
                "soc_drop_pct": energy_drop,
                "soc_after_pct": next_soc,
                "current_soc_pct": next_soc,
                "swaps_used_before": state.swaps_used,
                "swaps_used_after": state.swaps_used,
                "implicit_wait_min": max(0, service_start - state.time_min - outbound_min),
            }
            yield next_state, action, joint_duration, nest_duration


def reconstruct_actions(
    parent: dict[State, tuple[State, dict[str, Any]]],
    start: State,
    finish: State,
) -> list[dict[str, Any]]:
    actions: list[dict[str, Any]] = []
    state = finish
    while state != start:
        previous, action = parent[state]
        actions.append(action)
        state = previous
    actions.reverse()
    return actions


def solve(ctx: Context, max_labels: int = 500_000) -> dict[str, Any]:
    solve_start = perf_counter()
    all_tasks_mask = (1 << len(ctx.tasks)) - 1
    start = State(0, ctx.start_index, ctx.initial_soc, 0, 0)
    # 目标按完工时间、换电次数、无人机活动时间、车辆运行时间依次最小化。
    start_cost = (0, 0, 0, 0, 0)
    queue: list[tuple[tuple[int, int, int, int, int], int, State]] = []
    serial = count()
    heapq.heappush(queue, (start_cost, next(serial), start))
    best_by_core: dict[tuple[int, int, int, int], tuple[int, int, int, int, int]] = {state_core(start): start_cost}
    cost_by_state: dict[State, tuple[int, int, int, int, int]] = {start: start_cost}
    parent: dict[State, tuple[State, dict[str, Any]]] = {}
    expanded = 0
    generated = 1
    goal: State | None = None
    best_partial = start
    best_partial_rank = (0, 0, 0, 0, ctx.initial_soc, 0)

    task_weights = [int(task.get("priority_weight", 1)) for task in ctx.tasks]

    while queue:
        cost, _, state = heapq.heappop(queue)
        if cost_by_state.get(state) != cost:
            continue
        expanded += 1
        if expanded > max_labels:
            raise RuntimeError(f"状态标签超过上限{max_labels}，请缩小实例或增大--max-labels")

        completed_count = state.completed_mask.bit_count()
        completed_weight = sum(
            task_weights[index]
            for index in range(len(ctx.tasks))
            if state.completed_mask & (1 << index)
        )
        progress = -abs(ctx.end_index - state.station_index)
        partial_rank = (
            completed_count,
            completed_weight,
            int(state.station_index == ctx.end_index),
            progress,
            -state.time_min,
            state.soc_pct,
        )
        if partial_rank > best_partial_rank:
            best_partial_rank = partial_rank
            best_partial = state

        if state.completed_mask == all_tasks_mask and state.station_index == ctx.end_index:
            goal = state
            break

        for next_state, action, drone_delta, vehicle_delta in generate_transitions(ctx, state):
            next_cost = (
                next_state.time_min,
                next_state.swaps_used,
                cost[2] + drone_delta,
                cost[3] + vehicle_delta,
                cost[4] + 1,
            )
            core = state_core(next_state)
            previous_best = best_by_core.get(core)
            # 相同位置、SOC、任务集合和换电次数下，更早到达一定不劣。
            if previous_best is not None and previous_best <= next_cost:
                continue
            best_by_core[core] = next_cost
            cost_by_state[next_state] = next_cost
            parent[next_state] = (state, action)
            heapq.heappush(queue, (next_cost, next(serial), next_state))
            generated += 1

    finish = goal if goal is not None else best_partial
    actions = reconstruct_actions(parent, start, finish)
    completed_task_ids = [
        task["task_id"]
        for index, task in enumerate(ctx.tasks)
        if finish.completed_mask & (1 << index)
    ]
    uncompleted_task_ids = [
        task["task_id"] for task in ctx.tasks if task["task_id"] not in completed_task_ids
    ]
    total_task_count = len(ctx.tasks)
    completed_task_count = len(completed_task_ids)
    completion_rate_pct = round(completed_task_count / total_task_count * 100, 2)
    uav_task_assignments = [
        {
            "uav_id": action["uav_id"],
            "task_id": action["task_id"],
            "task_type": action["task_type"],
            "sortie_start_min": action["start_min"],
            "service_start_min": action["service_start_min"],
            "service_end_min": action["service_end_min"],
            "sortie_end_min": action["end_min"],
            "launch_station_id": action["uav_launch"],
            "recovery_station_id": action["uav_recovery"],
            "soc_before_pct": action["soc_before_pct"],
            "soc_after_pct": action["soc_after_pct"],
            "swaps_used_after": action["swaps_used_after"],
        }
        for action in actions
        if action["type"] == "joint_sortie_service"
    ]
    finish_cost = cost_by_state[finish]
    solve_runtime_ms = round((perf_counter() - solve_start) * 1000, 3)
    metadata = ctx.data.get("metadata", {})
    scenario_id = metadata.get("scenario_id", metadata.get("instance_id", ""))
    return {
        "run_config": {
            "scenario_id": scenario_id,
            "time_horizon_min": ctx.horizon,
            "time_step_min": ctx.time_step,
            "spare_battery_sets": ctx.spare_batteries,
            "allow_reverse": ctx.allow_reverse,
            "total_task_count": total_task_count,
            "uav_ids": [ctx.uav_id],
        },
        "status": "FEASIBLE_OPTIMAL" if goal is not None else "INFEASIBLE_ALL_TASKS_BEST_PARTIAL_RETURNED",
        "feasible": goal is not None,
        "solution_kind": "FULL" if goal is not None else "BEST_PARTIAL",
        "objective": {
            "finish_time_min": finish.time_min,
            "swaps_used": finish.swaps_used,
            "drone_active_time_min": finish_cost[2],
            "vehicle_travel_time_min": finish_cost[3],
        },
        "completed_task_ids": completed_task_ids,
        "uncompleted_task_ids": uncompleted_task_ids,
        "task_result": {
            "total_task_count": total_task_count,
            "completed_task_count": completed_task_count,
            "completion_rate_pct": completion_rate_pct,
            "completed_task_ids": completed_task_ids,
            "uncompleted_task_ids": uncompleted_task_ids,
        },
        "uav_task_assignments": uav_task_assignments,
        "uav_results": [
            {
                "uav_id": ctx.uav_id,
                "completed_task_count": completed_task_count,
                "completed_task_ids": completed_task_ids,
                "swaps_used": finish.swaps_used,
                "current_soc_pct": finish.soc_pct,
            }
        ],
        "final_state": {
            "time_min": finish.time_min,
            "station_id": ctx.stations[finish.station_index]["location_id"],
            "station_name": ctx.stations[finish.station_index]["name"],
            "soc_pct": finish.soc_pct,
            "current_soc_pct": finish.soc_pct,
            "swaps_used": finish.swaps_used,
            "cumulative_swaps_used": finish.swaps_used,
        },
        "runtime_ms": solve_runtime_ms,
        "actions": actions,
        "search_statistics": {
            "expanded_labels": expanded,
            "generated_labels": generated,
            "retained_state_cores": len(best_by_core),
        },
        "effective_parameters": {
            "spare_battery_sets": ctx.spare_batteries,
            "allow_reverse": ctx.allow_reverse,
            "time_horizon_min": ctx.horizon,
            "time_step_min": ctx.time_step,
            "soc_step_pct": ctx.soc_step,
            "uav_id": ctx.uav_id,
        },
    }


def validate_result(ctx: Context, result: dict[str, Any]) -> dict[str, Any]:
    """检查输出方案自身是否满足时间、任务、能量和换电一致性。"""
    violations: list[str] = []
    check_count = 0

    def require(condition: bool, message: str) -> None:
        nonlocal check_count
        check_count += 1
        if not condition:
            violations.append(message)

    actions = result["actions"]
    task_by_id = {task["task_id"]: task for task in ctx.tasks}
    known_uav_ids = {
        str(uav["uav_id"])
        for uav in ctx.data["resources"]["uavs"]
    }
    service_task_ids: list[str] = []
    previous_end = 0
    current_soc = ctx.initial_soc
    current_swaps = 0

    for step, action in enumerate(actions, start=1):
        prefix = f"动作{step}"
        start_min = int(action["start_min"])
        end_min = int(action["end_min"])
        soc_before = int(action["soc_before_pct"])
        soc_after = int(action["soc_after_pct"])
        swaps_before = int(action["swaps_used_before"])
        swaps_after = int(action["swaps_used_after"])

        require(start_min == previous_end, f"{prefix}开始时间与上一动作结束时间不连续")
        require(end_min >= start_min, f"{prefix}结束时间早于开始时间")
        require(action["uav_id"] in known_uav_ids, f"{prefix}引用未知无人机{action['uav_id']}")
        require(soc_before == current_soc, f"{prefix}动作前SOC与上一动作后SOC不一致")
        require(soc_after == int(action["current_soc_pct"]), f"{prefix}当前电量与动作后SOC不一致")
        require(ctx.reserve_soc <= soc_after <= ctx.initial_soc, f"{prefix}动作后SOC超出允许范围")
        require(swaps_before == current_swaps, f"{prefix}动作前换电次数与累计值不一致")

        if action["type"] == "battery_swap":
            require(swaps_after == swaps_before + 1, f"{prefix}换电动作没有使累计次数增加1")
            require(soc_after == ctx.initial_soc, f"{prefix}换电后SOC没有恢复到初始值")
        else:
            require(swaps_after == swaps_before, f"{prefix}非换电动作改变了累计换电次数")

        if action["type"] == "joint_sortie_service":
            task_id = action["task_id"]
            require(task_id in task_by_id, f"{prefix}引用未知任务{task_id}")
            if task_id in task_by_id:
                task = task_by_id[task_id]
                require(task_id not in service_task_ids, f"任务{task_id}被重复完成")
                require(
                    int(action["service_start_min"]) >= int(task["earliest_start_min"]),
                    f"任务{task_id}早于时间窗开始服务",
                )
                require(
                    int(action["service_end_min"]) <= int(task["latest_finish_min"]),
                    f"任务{task_id}晚于时间窗完成",
                )
                require(
                    start_min <= int(action["service_start_min"])
                    <= int(action["service_end_min"])
                    <= end_min,
                    f"任务{task_id}服务时间不在出动时间范围内",
                )
            service_task_ids.append(task_id)

        previous_end = end_min
        current_soc = soc_after
        current_swaps = swaps_after

    completed_task_ids = result["completed_task_ids"]
    assignments = result["uav_task_assignments"]
    assignment_pairs = [(row["uav_id"], row["task_id"]) for row in assignments]
    service_pairs = [
        (action["uav_id"], action["task_id"])
        for action in actions
        if action["type"] == "joint_sortie_service"
    ]

    require(set(service_task_ids) == set(completed_task_ids), "服务动作与已完成任务集合不一致")
    require(assignment_pairs == service_pairs, "无人机—任务对应关系与服务动作不一致")
    require(current_swaps <= ctx.spare_batteries, "累计换电次数超过备用电池数量")
    require(result["objective"]["swaps_used"] == current_swaps, "目标中的换电次数与动作累计值不一致")
    require(result["final_state"]["cumulative_swaps_used"] == current_swaps, "最终状态换电次数与动作累计值不一致")
    require(result["final_state"]["current_soc_pct"] == current_soc, "最终状态当前电量与最后动作不一致")
    require(result["final_state"]["time_min"] == previous_end, "最终状态时间与最后动作结束时间不一致")
    require(result["objective"]["finish_time_min"] == previous_end, "目标完工时间与最后动作结束时间不一致")
    require(previous_end <= ctx.horizon, "方案结束时间超过总时间窗口")
    require(
        result["task_result"]["completed_task_count"] == len(completed_task_ids),
        "任务汇总中的完成数量不一致",
    )
    require(
        result["task_result"]["total_task_count"] == len(ctx.tasks),
        "任务汇总中的总任务数不一致",
    )

    if result["feasible"]:
        require(len(completed_task_ids) == len(ctx.tasks), "完整可行解没有完成全部任务")
        require(
            result["final_state"]["station_id"] == ctx.stations[ctx.end_index]["location_id"],
            "完整可行解没有到达指定终点",
        )

    return {
        "passed": not violations,
        "check_count": check_count,
        "violations": violations,
    }


def find_minimum_spares(data: dict[str, Any], start_value: int, maximum: int, allow_reverse: bool) -> dict[str, Any]:
    trials: list[dict[str, Any]] = []
    for spare_count in range(start_value, maximum + 1):
        trial_context = build_context(
            data,
            spare_batteries_override=spare_count,
            allow_reverse=allow_reverse,
        )
        trial_result = solve(trial_context)
        trials.append(
            {
                "spare_battery_sets": spare_count,
                "feasible": trial_result["feasible"],
                "completed_task_count": len(trial_result["completed_task_ids"]),
                "finish_time_min": trial_result["objective"]["finish_time_min"],
            }
        )
        if trial_result["feasible"]:
            return {"minimum_feasible_spare_battery_sets": spare_count, "trials": trials}
    return {"minimum_feasible_spare_battery_sets": None, "trials": trials}


def write_outputs(result: dict[str, Any], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "summary.json"
    timeline_path = output_dir / "timeline.csv"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2)

    fieldnames = [
        "step",
        "type",
        "uav_id",
        "start_min",
        "end_min",
        "duration_min",
        "nest_from",
        "nest_to",
        "task_id",
        "task_type",
        "uav_launch",
        "uav_recovery",
        "service_start_min",
        "service_end_min",
        "flight_distance_m",
        "vehicle_distance_m",
        "drone_active_min",
        "vehicle_travel_min",
        "soc_before_pct",
        "soc_drop_pct",
        "soc_after_pct",
        "current_soc_pct",
        "swaps_used_before",
        "swaps_used_after",
        "implicit_wait_min",
        "details_json",
    ]
    with timeline_path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for index, action in enumerate(result["actions"], start=1):
            writer.writerow(
                {
                    "step": index,
                    "type": action.get("type", ""),
                    "uav_id": action.get("uav_id", ""),
                    "start_min": action.get("start_min", ""),
                    "end_min": action.get("end_min", ""),
                    "duration_min": action.get("end_min", 0) - action.get("start_min", 0),
                    "nest_from": action.get("nest_from", ""),
                    "nest_to": action.get("nest_to", ""),
                    "task_id": action.get("task_id", ""),
                    "task_type": action.get("task_type", ""),
                    "uav_launch": action.get("uav_launch", ""),
                    "uav_recovery": action.get("uav_recovery", ""),
                    "service_start_min": action.get("service_start_min", ""),
                    "service_end_min": action.get("service_end_min", ""),
                    "flight_distance_m": action.get("flight_distance_m", ""),
                    "vehicle_distance_m": action.get("vehicle_distance_m", ""),
                    "drone_active_min": action.get("drone_active_min", ""),
                    "vehicle_travel_min": action.get("vehicle_travel_min", ""),
                    "soc_before_pct": action.get("soc_before_pct", ""),
                    "soc_drop_pct": action.get("soc_drop_pct", ""),
                    "soc_after_pct": action.get("soc_after_pct", ""),
                    "current_soc_pct": action.get("current_soc_pct", ""),
                    "swaps_used_before": action.get("swaps_used_before", ""),
                    "swaps_used_after": action.get("swaps_used_after", ""),
                    "implicit_wait_min": action.get("implicit_wait_min", ""),
                    "details_json": json.dumps(action, ensure_ascii=False, separators=(",", ":")),
                }
            )


def parse_args() -> argparse.Namespace:
    script_dir = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="昌平线车—机协同巡检最小实例基准求解器")
    parser.add_argument(
        "--input",
        type=Path,
        default=script_dir / "data" / "changping_feasible_demo.json",
        help="实例JSON路径",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=script_dir / "results",
        help="结果目录",
    )
    parser.add_argument("--spare-batteries", type=int, default=None, help="覆盖数据中的备用电池组数")
    parser.add_argument("--time-horizon", type=int, default=None, help="覆盖总时间窗口，并同步调整班次及贴近原时域的任务截止时间")
    parser.add_argument("--allow-reverse", action="store_true", help="允许移动机巢反向运行")
    parser.add_argument("--diagnose-max-spares", type=int, default=6, help="无解时将备用电池敏感性检查到该数值")
    parser.add_argument("--max-labels", type=int, default=500_000, help="最大状态标签数")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data = load_instance(args.input)
    if args.time_horizon is not None:
        apply_time_horizon_override(data, args.time_horizon)
    context = build_context(
        data,
        spare_batteries_override=args.spare_batteries,
        allow_reverse=args.allow_reverse,
    )
    result = solve(context, max_labels=args.max_labels)
    result["validation"] = validate_result(context, result)
    if not result["feasible"] and args.diagnose_max_spares >= context.spare_batteries:
        result["battery_sensitivity"] = find_minimum_spares(
            data,
            context.spare_batteries,
            args.diagnose_max_spares,
            args.allow_reverse,
        )
    write_outputs(result, args.output_dir)

    print(f"状态: {result['status']}")
    print(f"完成任务: {len(result['completed_task_ids'])}/{len(context.tasks)}")
    print(f"结束时刻: {result['objective']['finish_time_min']} min")
    print(f"换电次数: {result['objective']['swaps_used']}")
    print(f"结果目录: {args.output_dir.resolve()}")
    if not result["feasible"]:
        sensitivity = result.get("battery_sensitivity", {})
        minimum = sensitivity.get("minimum_feasible_spare_battery_sets")
        if minimum is not None:
            print(f"诊断: 在其他参数不变时，至少需要{minimum}组备用电池才能覆盖全部任务。")
        else:
            print("诊断: 在当前敏感性范围内仍无全任务可行解。")
    return 0 if result["feasible"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
