from __future__ import annotations

import asyncio
import ast
import sys
import unittest
from pathlib import Path
from unittest.mock import call, patch

from fastapi import HTTPException


PLC2_DIR = Path(__file__).resolve().parents[1]
if str(PLC2_DIR) not in sys.path:
    sys.path.insert(0, str(PLC2_DIR))


from api.main import (  # noqa: E402
    WriteByPointBody,
    get_registers_by_points,
    write_by_point,
)
from service.plc_service import (  # noqa: E402
    PLC_SERVICE,
    PlcPointValidationError,
    PlcService,
)


class PointControlApiTests(unittest.TestCase):
    def test_ui_flows_and_device_services_do_not_write_raw_addresses(self) -> None:
        files = [PLC2_DIR / "ui" / "main.py"]
        files.extend((PLC2_DIR / "flow").glob("*.py"))
        files.extend(
            path
            for path in (PLC2_DIR / "service").glob("*.py")
            if path.name != "plc_service.py"
        )

        violations: list[str] = []
        for path in files:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                    continue
                if node.func.attr in {"write_d_register", "write_bit_device"}:
                    violations.append(f"{path.relative_to(PLC2_DIR)}:{node.lineno}")

        self.assertEqual([], violations)

    def test_point_write_validation_happens_before_hardware_access(self) -> None:
        service = PlcService()

        with self.assertRaises(PlcPointValidationError):
            service.write_point("X_CUR_POS", 10)
        with self.assertRaises(PlcPointValidationError):
            service.write_point("X_FWD_POS", 1451)
        with self.assertRaises(PlcPointValidationError):
            service.write_point("X_UP", "not-a-switch")

        self.assertFalse(service.is_connected())

    def test_point_api_delegates_reads_and_writes_to_shared_service(self) -> None:
        with patch.object(
            PLC_SERVICE,
            "read_point",
            side_effect=[321, False],
        ) as read_point:
            response = asyncio.run(
                get_registers_by_points(
                    ids=["X_CUR_POS", "X_UP"],
                    plc=None,
                    user="test_user",
                )
            )

        self.assertEqual(
            [
                {"point_id": "X_CUR_POS", "value": 321.0},
                {"point_id": "X_UP", "value": False},
            ],
            response.data,
        )
        self.assertEqual(
            [call("X_CUR_POS"), call("X_UP")],
            read_point.call_args_list,
        )

        with patch.object(PLC_SERVICE, "write_point") as write_point:
            response = asyncio.run(
                write_by_point(
                    WriteByPointBody(point_id="X_SPEED", value=250),
                    plc=None,
                    user="test_user",
                )
            )

        write_point.assert_called_once_with("X_SPEED", 250)
        self.assertEqual("X_SPEED", response.data["point_id"])

    def test_point_api_rejects_invalid_values_without_connecting(self) -> None:
        with self.assertRaises(HTTPException) as context:
            asyncio.run(
                write_by_point(
                    WriteByPointBody(point_id="X_SPEED", value=0),
                    plc=None,
                    user="test_user",
                )
            )

        self.assertEqual(400, context.exception.status_code)
        self.assertFalse(PLC_SERVICE.is_connected())

    def test_point_api_rejects_plc_mapping_override(self) -> None:
        with self.assertRaises(HTTPException) as context:
            asyncio.run(
                get_registers_by_points(
                    ids=["X_CUR_POS"],
                    plc="other_plc",
                    user="test_user",
                )
            )

        self.assertEqual(400, context.exception.status_code)


if __name__ == "__main__":
    unittest.main()
