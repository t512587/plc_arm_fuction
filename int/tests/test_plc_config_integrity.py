from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from plc.service_plc import load_points


class PlcConfigIntegrityTests(unittest.TestCase):
    def test_active_config_allows_only_the_documented_d240_alias(self) -> None:
        points = load_points()
        by_address: dict[tuple[str, int], set[str]] = {}
        for point in points:
            by_address.setdefault((point.device, point.address), set()).add(point.id)

        duplicates = {key: ids for key, ids in by_address.items() if len(ids) > 1}
        self.assertEqual(
            {("D", 240): {"y1_forward_speed", "y2_forward_speed"}},
            duplicates,
        )

    def test_unapproved_duplicate_address_fails_at_startup(self) -> None:
        content = """points:
  - {id: first, name: First, device: M, address: 10}
  - {id: second, name: Second, device: M, address: 10}
"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "points.yml"
            path.write_text(content, encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "M10 被重複定義"):
                load_points(path)


if __name__ == "__main__":
    unittest.main()
