import unittest
from pathlib import Path

from changping_min_solver import build_context, load_instance, solve, validate_instance


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

    def test_full_instance_is_reported_infeasible(self) -> None:
        data = load_instance(ROOT / "data" / "changping_min_instance.json")
        result = solve(build_context(data))
        self.assertFalse(result["feasible"])
        self.assertGreaterEqual(len(result["completed_task_ids"]), 3)


if __name__ == "__main__":
    unittest.main()
