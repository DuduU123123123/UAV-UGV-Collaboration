import copy
import unittest
from pathlib import Path

from changping_min_solver import (
    apply_time_horizon_override,
    build_context,
    load_instance,
    solve,
    validate_instance,
    validate_result,
)
from run_experiments import build_manifest_row


ROOT = Path(__file__).resolve().parent


class ChangpingMinimumSolverTests(unittest.TestCase):
    def test_base_instance_structure(self) -> None:
        data = load_instance(ROOT / "data" / "changping_min_instance.json")
        validate_instance(data)
        self.assertEqual(len(data["stations"]), 6)
        self.assertEqual(len(data["segments"]), 5)
        self.assertEqual(len(data["directed_vehicle_arcs"]), 10)
        self.assertEqual(sum(row["distance_m"] for row in data["segments"]), 10797)

    def test_default_demo_is_feasible(self) -> None:
        data = load_instance(ROOT / "data" / "changping_feasible_demo.json")
        result = solve(build_context(data))
        self.assertTrue(result["feasible"])
        self.assertEqual(set(result["completed_task_ids"]), {"T01", "T03", "T04"})
        self.assertEqual(result["final_state"]["station_id"], "S06")
        self.assertLessEqual(result["objective"]["finish_time_min"], 120)
        self.assertLessEqual(result["objective"]["swaps_used"], 2)
        self.assertEqual(result["solution_kind"], "FULL")
        self.assertEqual(result["task_result"]["completed_task_count"], 3)
        self.assertEqual(result["task_result"]["completion_rate_pct"], 100.0)
        self.assertEqual(
            [assignment["task_id"] for assignment in result["uav_task_assignments"]],
            ["T01", "T03", "T04"],
        )
        self.assertTrue(
            all(assignment["uav_id"] == "U01" for assignment in result["uav_task_assignments"])
        )
        self.assertEqual(result["uav_results"][0]["swaps_used"], 2)
        self.assertEqual(
            result["uav_results"][0]["current_soc_pct"],
            result["final_state"]["soc_pct"],
        )
        self.assertGreaterEqual(result["runtime_ms"], 0)

    def test_full_instance_is_reported_infeasible(self) -> None:
        data = load_instance(ROOT / "data" / "changping_min_instance.json")
        result = solve(build_context(data))
        self.assertFalse(result["feasible"])
        self.assertGreaterEqual(len(result["completed_task_ids"]), 3)
        self.assertEqual(result["solution_kind"], "BEST_PARTIAL")
        self.assertEqual(
            result["task_result"]["completed_task_count"],
            len(result["uav_task_assignments"]),
        )

    def test_actions_report_current_soc_and_cumulative_swaps(self) -> None:
        data = load_instance(ROOT / "data" / "changping_feasible_demo.json")
        result = solve(build_context(data))
        self.assertTrue(result["actions"])
        for action in result["actions"]:
            self.assertEqual(action["uav_id"], "U01")
            self.assertEqual(action["current_soc_pct"], action["soc_after_pct"])
            self.assertIn("swaps_used_after", action)
        self.assertEqual(
            result["final_state"]["cumulative_swaps_used"],
            result["objective"]["swaps_used"],
        )

    def test_result_validation_accepts_generated_solutions(self) -> None:
        for filename in ("changping_feasible_demo.json", "changping_min_instance.json"):
            data = load_instance(ROOT / "data" / filename)
            context = build_context(data)
            validation = validate_result(context, solve(context))
            self.assertTrue(validation["passed"], validation["violations"])
            self.assertGreater(validation["check_count"], 0)
            self.assertEqual(validation["violations"], [])

    def test_result_validation_detects_inconsistent_current_soc(self) -> None:
        data = load_instance(ROOT / "data" / "changping_feasible_demo.json")
        context = build_context(data)
        result = copy.deepcopy(solve(context))
        result["actions"][0]["current_soc_pct"] = 999
        validation = validate_result(context, result)
        self.assertFalse(validation["passed"])
        self.assertTrue(
            any("当前电量与动作后SOC不一致" in message for message in validation["violations"])
        )

    def test_time_horizon_override_updates_linked_deadlines(self) -> None:
        data = load_instance(ROOT / "data" / "changping_min_instance.json")
        apply_time_horizon_override(data, 135)
        validate_instance(data)
        self.assertEqual(data["parameters"]["time_horizon_min"], 135)
        self.assertEqual(data["resources"]["mobile_nests"][0]["shift_end_min"], 135)
        self.assertEqual(
            {task["task_id"]: task["latest_finish_min"] for task in data["tasks"]},
            {"T01": 60, "T02": 90, "T03": 100, "T04": 135, "T05": 135},
        )

    def test_experiment_manifest_contains_report_fields(self) -> None:
        data = load_instance(ROOT / "data" / "changping_feasible_demo.json")
        context = build_context(data)
        result = solve(context)
        result["validation"] = validate_result(context, result)
        row = build_manifest_row("test_h120_b02", result)
        expected_fields = {
            "experiment_id",
            "time_horizon_min",
            "spare_battery_sets",
            "feasible",
            "total_task_count",
            "completed_task_count",
            "completion_rate_pct",
            "uav_task_mapping",
            "finish_time_min",
            "swaps_used",
            "current_soc_pct",
            "runtime_ms",
            "validation_passed",
        }
        self.assertTrue(expected_fields.issubset(row))
        self.assertEqual(row["uav_task_mapping"], ["U01:T01", "U01:T03", "U01:T04"])


if __name__ == "__main__":
    unittest.main()
