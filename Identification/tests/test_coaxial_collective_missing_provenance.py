"""Fail-closed provenance checks for offline coaxial LQI evaluation."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from flight_identification import coaxial_collective_evaluation as evaluation


class CoaxialCollectiveMissingProvenanceTest(unittest.TestCase):
    def test_missing_report_requires_explicit_offline_opt_in(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            sim = root / "SimEnv/configs/plant.yaml"
            controller = root / "Controller/configs/lqi.yaml"
            calibration = root / "log/motor.csv"
            header = root / "px4_trans/generated/airframe-1/LqiIdentifiedModel.hpp"
            for file in (sim, controller, calibration, header):
                file.parent.mkdir(parents=True, exist_ok=True)
                file.write_text("test fixture\\n", encoding="utf-8")
            header.write_text("0x1716d847u\\n", encoding="utf-8")
            model = {
                "airframe": {
                    "id": "airframe-1",
                    "identification_report": "Identification/datasets/missing.json",
                    "upper_motor_speed_calibration": "log/motor.csv",
                    "identified_lqi_checksum": "0x1716d847",
                    "upper_motor_calibration_domain": "physical_command_times_two",
                }
            }

            with patch.object(evaluation, "ROOT", root):
                with self.assertRaisesRegex(FileNotFoundError, "offline simulation only"):
                    evaluation._provenance(sim, controller, model)

                provenance = evaluation._provenance(
                    sim,
                    controller,
                    model,
                    allow_missing_identification_report=True,
                )
                self.assertEqual(provenance["airframe_id"], "airframe-1")
                self.assertIsNone(provenance["airframe_number"])
                self.assertIsNone(provenance["controller_contract"])
                self.assertFalse(provenance["identification_report_available"])
                self.assertIsNone(provenance["identification_report_sha256"])
                self.assertEqual(
                    provenance["airframe_identity_source"], "controller_config_only"
                )
                self.assertTrue(provenance["checksum_present_in_embedded_header"])

                report_path = root / "Identification/datasets/missing.json"
                report_path.parent.mkdir(parents=True, exist_ok=True)
                report_path.write_text(
                    json.dumps(
                        {
                            "airframe": {
                                "airframe_id": "airframe-1",
                                "airframe_number": 1,
                                "controller_contract": "archived-test-contract",
                            }
                        }
                    ),
                    encoding="utf-8",
                )
                verified = evaluation._provenance(sim, controller, model)
                self.assertTrue(verified["identification_report_available"])
                self.assertEqual(verified["airframe_number"], 1)
                self.assertEqual(
                    verified["airframe_identity_source"], "identification_report"
                )
                self.assertIsNotNone(verified["identification_report_sha256"])


if __name__ == "__main__":
    unittest.main()
