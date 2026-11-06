"""Airframe-3 fail-closed checks; no simulator or flight hardware required."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]
GENERATOR = ROOT / "px4_trans/tools/generate_coaxial_collective_backend.py"
CONFIG = ROOT / "px4_trans/configs/lqi_coaxial_collective_airframe3.yaml"


class Airframe3DeploymentPreflightTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        spec = importlib.util.spec_from_file_location("airframe3_coaxial_generator", GENERATOR)
        assert spec and spec.loader
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_airframe3_cannot_generate_without_independent_calibration(self) -> None:
        with self.assertRaisesRegex(ValueError, "uncalibrated"):
            self.module.load_model(CONFIG)

    def test_new_airframe_metadata_remains_inhibited(self) -> None:
        deployment = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        self.assertEqual(deployment["airframe_id"], "airframe-3")
        self.assertIs(deployment["calibration_ready"], False)
        self.assertIs(deployment["flight_authorized"], False)
        self.assertIsNone(deployment["identified_model"]["expected_checksum"])
        self.assertIsNone(deployment["physical_motor_pole_count"])
        self.assertNotEqual(
            deployment["motor_speed_calibration"]["path"],
            deployment["lower_motor_speed_calibration"]["path"],
        )

    def test_independent_airframe3_registration(self) -> None:
        from flight_identification.airframe import resolve_airframe
        airframe = resolve_airframe("airframe-3")
        self.assertEqual(airframe["airframe_number"], 3)
        self.assertEqual(airframe["status"], "awaiting_independent_calibration")
        self.assertFalse(airframe["identification"]["flight_authorized"])


if __name__ == "__main__":
    unittest.main()
