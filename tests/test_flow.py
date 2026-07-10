from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from engine import flow


class FlowTest(unittest.TestCase):
    def test_astar_stage_uses_memory_inputs(self) -> None:
        case = flow.load_case(flow.ROOT / "cases" / "toy_case" / "path.yml")
        matching = flow.matching_net_indices(case)
        matching_case = flow.subset_case(case, matching, matching_only=True)

        with tempfile.TemporaryDirectory() as tmp:
            paths = flow.run_astar(Path(tmp), matching_case)
            files = {
                path.relative_to(tmp).as_posix()
                for path in Path(tmp).rglob("*")
                if path.is_file()
            }

        self.assertEqual(
            files,
            {"01_astar/astar_paths.txt", "01_astar/astar_order.json"},
        )
        self.assertEqual(
            flow.endpoint_signature(paths),
            flow.endpoint_signature(
                [
                    [flow.to_point(net["source"]), flow.to_point(net["target"])]
                    for net in matching_case["endpoints"]
                ]
            ),
        )


if __name__ == "__main__":
    unittest.main()
