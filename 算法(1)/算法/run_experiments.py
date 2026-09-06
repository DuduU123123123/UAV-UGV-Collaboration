#!/usr/bin/env python3
"""批量运行时间窗口×备用电池数量敏感性实验。"""

from __future__ import annotations

import copy
import csv
import json
from pathlib import Path
from typing import Any

from changping_min_solver import (
    apply_time_horizon_override,
    build_context,
    load_instance,
    solve,
    validate_result,
    write_outputs,
)


ROOT = Path(__file__).resolve().parent
INPUT_PATH = ROOT / "data" / "changping_min_instance.json"
OUTPUT_ROOT = ROOT / "experiments"
HORIZONS = (120, 125, 135, 150)
BATTERY_COUNTS = (2, 3, 4, 5)


def build_manifest_row(experiment_id: str, result: dict[str, Any]) -> dict[str, Any]:
    task_result = result["task_result"]
    objective = result["objective"]
    final_state = result["final_state"]
    return {
        "experiment_id": experiment_id,
        "time_horizon_min": result["run_config"]["time_horizon_min"],
        "spare_battery_sets": result["run_config"]["spare_battery_sets"],
        "allow_reverse": result["run_config"]["allow_reverse"],
        "status": result["status"],
        "feasible": result["feasible"],
        "solution_kind": result["solution_kind"],
        "total_task_count": task_result["total_task_count"],
        "completed_task_count": task_result["completed_task_count"],
        "completion_rate_pct": task_result["completion_rate_pct"],
        "completed_task_ids": task_result["completed_task_ids"],
        "uncompleted_task_ids": task_result["uncompleted_task_ids"],
        "uav_task_mapping": [
            f"{row['uav_id']}:{row['task_id']}"
            for row in result["uav_task_assignments"]
        ],
        "finish_time_min": objective["finish_time_min"],
        "swaps_used": objective["swaps_used"],
        "drone_active_time_min": objective["drone_active_time_min"],
        "vehicle_travel_time_min": objective["vehicle_travel_time_min"],
        "current_soc_pct": final_state["current_soc_pct"],
        "runtime_ms": result["runtime_ms"],
        "expanded_labels": result["search_statistics"]["expanded_labels"],
        "generated_labels": result["search_statistics"]["generated_labels"],
        "validation_passed": result["validation"]["passed"],
        "validation_check_count": result["validation"]["check_count"],
        "output_directory": experiment_id,
    }


def write_experiment_matrix(manifest: list[dict[str, Any]], output_path: Path) -> None:
    rows: list[dict[str, Any]] = []
    for manifest_row in manifest:
        row = dict(manifest_row)
        row["completed_task_ids"] = ";".join(row["completed_task_ids"])
        row["uncompleted_task_ids"] = ";".join(row["uncompleted_task_ids"])
        row["uav_task_mapping"] = ";".join(row["uav_task_mapping"])
        rows.append(row)

    with output_path.open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_all_experiments() -> list[dict[str, Any]]:
    base_data = load_instance(INPUT_PATH)
    manifest: list[dict[str, Any]] = []

    for horizon in HORIZONS:
        for battery_count in BATTERY_COUNTS:
            experiment_id = f"h{horizon}_b{battery_count:02d}"
            data = copy.deepcopy(base_data)
            apply_time_horizon_override(data, horizon)
            context = build_context(data, spare_batteries_override=battery_count)
            result = solve(context)
            result["experiment_id"] = experiment_id
            result["run_config"]["deadline_adjustment_policy"] = (
                "同步调整原本等于基础原总时域的任务截止时间"
            )
            result["validation"] = validate_result(context, result)
            if not result["validation"]["passed"]:
                raise RuntimeError(
                    f"实验{experiment_id}未通过结果校验: "
                    f"{result['validation']['violations']}"
                )

            write_outputs(result, OUTPUT_ROOT / experiment_id)
            manifest.append(build_manifest_row(experiment_id, result))

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    with (OUTPUT_ROOT / "run_manifest.json").open("w", encoding="utf-8") as handle:
        json.dump(manifest, handle, ensure_ascii=False, indent=2)
    write_experiment_matrix(manifest, OUTPUT_ROOT / "experiment_matrix.csv")
    return manifest


def main() -> int:
    manifest = run_all_experiments()
    feasible_count = sum(1 for row in manifest if row["feasible"])
    print(f"完成实验: {len(manifest)}组")
    print(f"全任务可行: {feasible_count}组")
    print(f"综合CSV: {(OUTPUT_ROOT / 'experiment_matrix.csv').resolve()}")
    print(f"结果目录: {OUTPUT_ROOT.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
