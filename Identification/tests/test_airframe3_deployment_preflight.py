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

    def test_archive_identity_rebinding_changes_only_embedded_identity(self) -> None:
        # Archived payload is the authoritative numeric model: DARE re-solving
        # is not reproducible byte-for-byte across all numerical environments.
        import sys
        import importlib.util
        helper_path = ROOT / "px4_trans/tools/generate_airframe3_baseline.py"
        tools_path = str(helper_path.parent)
        sys.path.insert(0, tools_path)
        try:
            spec = importlib.util.spec_from_file_location(
                "airframe3_baseline_helper", helper_path
            )
            assert spec and spec.loader
            helper = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(helper)
            config = yaml.safe_load((
                ROOT / "Identification/configs/airframe3_baseline_from_airframe1.yaml"
            ).read_text(encoding="utf-8"))
            manifest, sources = helper.load_verified_source(
                config["expected_source_model_checksum"]
            )
            original = sources["payload"].read_text(encoding="utf-8")
            replacement = helper.rebind_identity_only(original)
            self.assertIn("kAirframeNumber{3u}", replacement)
            self.assertIn('kAirframeId[]{"airframe-3"}', replacement)
            self.assertNotIn("kAirframeNumber{1u}", replacement)
            self.assertEqual(
                replacement.split("namespace lqi_coaxial_collective")[1].replace(
                    "kAirframeNumber{3u}", "kAirframeNumber{1u}"
                ).replace("airframe-3", "airframe-1"),
                original.split("namespace lqi_coaxial_collective")[1],
            )
            self.assertEqual(manifest["model_checksum"], "0x78ae1111")
        finally:
            sys.path.remove(tools_path)

    def test_airframe3_cannot_generate_without_independent_calibration(self) -> None:
        with self.assertRaisesRegex(ValueError, "uncalibrated"):
            self.module.load_model(CONFIG)

    def test_normal_airframe1_generator_requires_real_report(self) -> None:
        # The fallback is opt-in only for the assumed-parameter baseline.
        source = ROOT / "px4_trans/configs/lqi_coaxial_collective_airframe1_manual_v3.yaml"
        config = yaml.safe_load(source.read_text(encoding="utf-8"))
        report_path = ROOT / config["identified_model"]["report"]
        if report_path.is_file():
            self.skipTest("historical report is available on this checkout")
        with self.assertRaisesRegex(FileNotFoundError, "identification report"):
            self.module.load_model(source)

    def test_missing_report_exception_rejected_for_airframe3(self) -> None:
        with self.assertRaisesRegex(ValueError, "only for airframe-1"):
            self.module.load_model(
                CONFIG, allow_missing_identification_report=True
            )

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
