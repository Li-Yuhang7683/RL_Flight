from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from flight_identification.airframe import (
    REPOSITORY_ROOT,
    airframe_provenance,
    load_airframe_registry,
    read_bundle_airframe,
    resolve_airframe,
    validate_airframe_payload,
    write_airframe_sidecar,
)


class AirframeProvenanceTest(unittest.TestCase):
    def test_current_dataset_and_generated_model_are_bound_to_airframe_1(self) -> None:
        identity = read_bundle_airframe(
            REPOSITORY_ROOT
            / "Identification/datasets/real_logs_26_10_26_v4/bundle.npz"
        )
        self.assertIsNotNone(identity)
        self.assertEqual(identity["airframe_id"], "airframe-1")
        header = (
            REPOSITORY_ROOT
            / "px4_trans/px4/src/modules/nn_control/LqiIdentifiedModel.hpp"
        ).read_text(encoding="utf-8")
        self.assertIn('kAirframeId[]{"airframe-1"}', header)
        self.assertIn("static constexpr uint32_t kAirframeNumber{1u}", header)

    def test_registry_resolves_stable_identity(self) -> None:
        registry = load_airframe_registry()
        first = resolve_airframe("airframe-1")
        second = resolve_airframe("airframe-2")
        self.assertEqual(first["airframe_number"], 1)
        self.assertEqual(second["airframe_number"], 2)
        self.assertEqual(airframe_provenance(first)["display_name"], "机架1")
        with self.assertRaisesRegex(ValueError, "unknown airframe_id"):
            resolve_airframe("airframe-999")

    def test_bundle_sidecar_and_embedded_identity_are_checked(self) -> None:
        airframe = airframe_provenance(resolve_airframe("airframe-1"))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            write_airframe_sidecar(
                root,
                resolve_airframe("airframe-1"),
                artifact_type="test_dataset",
            )
            bundle = root / "bundle.npz"
            np.savez(
                bundle,
                features=np.zeros((1, 2, 1), dtype=np.float32),
                airframe_id=np.asarray(airframe["airframe_id"]),
                airframe_json=np.asarray(
                    json.dumps(airframe, ensure_ascii=False)
                ),
            )
            self.assertEqual(
                read_bundle_airframe(bundle)["airframe_id"], "airframe-1"
            )

            np.savez(
                bundle,
                features=np.zeros((1, 2, 1), dtype=np.float32),
                airframe_id=np.asarray("airframe-2"),
                airframe_json=np.asarray(
                    json.dumps(airframe, ensure_ascii=False)
                ),
            )
            with self.assertRaisesRegex(ValueError, "bundle airframe_id"):
                read_bundle_airframe(bundle)

    def test_report_airframe_mismatch_is_rejected(self) -> None:
        payload = {"airframe": airframe_provenance(resolve_airframe("airframe-1"))}
        self.assertEqual(
            validate_airframe_payload(
                payload, "airframe-1", artifact_name="report"
            )["airframe_id"],
            "airframe-1",
        )
        with self.assertRaisesRegex(ValueError, "belongs to"):
            validate_airframe_payload(
                payload, "airframe-2", artifact_name="report"
            )


if __name__ == "__main__":
    unittest.main()
