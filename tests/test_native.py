from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

from engine.native import route


class NativeRoutingTest(unittest.TestCase):
    def test_route_returns_path_in_memory(self) -> None:
        previous_cwd = Path.cwd()
        with tempfile.TemporaryDirectory() as tmp:
            os.chdir(tmp)
            try:
                result = route(
                    [
                        {
                            "source": (0, 0),
                            "target": (3, 0),
                            "source_direction": 1,
                            "target_direction": 3,
                        }
                    ],
                    4,
                    4,
                    [],
                    {"pitch": 10.0, "order": "input"},
                )
                self.assertEqual(list(Path(tmp).iterdir()), [])
            finally:
                os.chdir(previous_cwd)

        self.assertEqual(len(result["routes"]), 1)
        endpoints = {tuple(result["routes"][0][0]), tuple(result["routes"][0][-1])}
        self.assertEqual(endpoints, {(0, 0), (3, 0)})
        self.assertGreaterEqual(result["total_cost"], 0.0)


if __name__ == "__main__":
    unittest.main()
