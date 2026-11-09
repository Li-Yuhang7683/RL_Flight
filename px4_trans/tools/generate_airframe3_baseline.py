#!/usr/bin/env python3
"""Prepare an airframe-3 *inhibited bench candidate* from airframe-1 manual-v3.

No independent airframe-3 identification is claimed. This script reuses the
airframe-1 nominal physical values deliberately, but rebinds the payload's
embedded airframe identity to 3 and archives explicit provenance.

Default: archive payload and golden vectors only. No PX4 active file changes.
Only --stage-for-bench-build copies artifacts into the PX4 source tree.
That flag DOES NOT enable actuator outputs, or grant any flight clearance.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil

import yaml

import generate_coaxial_collective_backend as generator

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "Identification/configs/airframe3_baseline_from_airframe1.yaml"


def prepare(config_path: Path, *, stage_for_bench_build: bool = False) -> dict:
    from flight_identification.airframe import resolve_airframe

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        raise ValueError("unsupported baseline generation schema")
    if (config.get("target_airframe_id"), config.get("source_airframe_id")) != (
        "airframe-3", "airframe-1"
    ):
        raise ValueError("baseline requires airframe-1 provenance and airframe-3 identity")
    if config.get("flight_authorized") is not False:
        raise ValueError("baseline must remain flight_authorized=false")
    if config.get("independently_calibrated") is not False:
        raise ValueError("baseline must not claim airframe-3 calibration")
    if config.get("parameter_provenance") != (
        "assumed_identical_to_airframe1_manual_v3"
    ):
        raise ValueError("assumption provenance must be explicit")
    if config.get("purpose") != (
        "offline_simulation_and_inhibited_bench_firmware_only"
    ):
        raise ValueError("bench-only purpose must be explicit")

    record = resolve_airframe(
        config["target_airframe_id"], ROOT / config["airframe_registry"]
    )
    if int(record["airframe_number"]) != 3:
        raise ValueError("airframe registry number must be 3")
    if record["controller_contract"] != (
        "pilot_acceleration_dual_motor_common_differential_servo_roll_pitch"
    ):
        raise ValueError("incompatible airframe-3 controller contract")
    if record["identification"].get("flight_authorized") is not False:
        raise ValueError("airframe-3 registry must not authorize flight")

    source_path = (ROOT / config["source_deployment_config"]).resolve()
    approved_source = (
        ROOT / "px4_trans/configs/lqi_coaxial_collective_airframe1_manual_v3.yaml"
    ).resolve()
    if source_path != approved_source:
        raise ValueError("baseline must use the pinned airframe-1 manual-v3 configuration")
    source_description = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    if source_description.get("airframe_id") != "airframe-1":
        raise ValueError("baseline source is not airframe-1")

    model, _, source_metadata = generator.load_model(source_path)
    digest = generator.checksum(model)
    if config["expected_source_model_checksum"].lower() != f"0x{digest:08x}":
        raise ValueError("airframe-1 source model checksum changed; review required")

    # Identity and provenance are distinct: never change the source data silently.
    metadata = {
        **source_metadata,
        "airframe_id": "airframe-3",
        "airframe_number": 3,
        "platform_family": record["platform_family"],
        "baseline_source_airframe_id": "airframe-1",
        "parameter_provenance": config["parameter_provenance"],
        "independently_calibrated": False,
        "flight_authorized": False,
    }
    output_dir = (ROOT / config["output_dir"]).resolve()
    approved_dir = (ROOT / "px4_trans/generated/airframe-3").resolve()
    if not output_dir.is_relative_to(approved_dir):
        raise ValueError("baseline artifacts must remain in airframe-3 generated directory")
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = output_dir / "LqiCoaxialCollectivePayload.hpp"
    golden_h = output_dir / "lqi_coaxial_collective_golden.hpp"
    golden_json = output_dir / "lqi_coaxial_collective_golden.json"
    manifest_path = output_dir / "lqi_coaxial_collective_manifest.json"

    generator.write_model_header(model, metadata, payload)
    vectors = generator.golden_vectors(model)
    generator.write_golden(vectors, golden_h, golden_json)
    manifest = {
        "schema_version": 1,
        "artifact_type": "assumed_parameter_coaxial_lqi_bench_candidate",
        "target_airframe_id": "airframe-3",
        "target_airframe_number": 3,
        "source_airframe_id": "airframe-1",
        "parameter_provenance": config["parameter_provenance"],
        "independently_calibrated": False,
        "flight_authorized": False,
        "actuator_output_authorized": False,
        "purpose": config["purpose"],
        "model_checksum": f"0x{digest:08x}",
        "source": metadata,
        "files": {
            "payload": str(payload.relative_to(ROOT)),
            "payload_sha256": generator.file_sha256(payload),
            "golden_header": str(golden_h.relative_to(ROOT)),
            "golden_json": str(golden_json.relative_to(ROOT)),
        },
        "staged_for_bench_build": stage_for_bench_build,
        "notes": [
            "This package uses airframe-1 manual-v3 nominal parameters.",
            "The identical checksum denotes identical model numbers, not independently identified airframe-3 physics.",
            "Model outputs are NOT flight-authorized.",
        ],
    }
    if stage_for_bench_build:
        for candidate, active in (
            (payload, generator.PAYLOAD_HEADER),
            (golden_h, generator.GOLDEN_HEADER),
            (golden_json, generator.GOLDEN_JSON),
        ):
            backup = output_dir / ("previous_active_" + active.name)
            if active.exists():
                shutil.copyfile(active, backup)
            shutil.copyfile(candidate, active)
            manifest["files"]["staged_" + active.name + "_sha256"] = generator.file_sha256(active)
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--stage-for-bench-build", action="store_true",
        help="EXPLICITLY stage candidate payload for INHIBITED bench firmware build",
    )
    args = parser.parse_args()
    manifest = prepare(
        args.config.resolve(), stage_for_bench_build=args.stage_for_bench_build
    )
    print(json.dumps({
        "target_airframe_id": manifest["target_airframe_id"],
        "model_checksum": manifest["model_checksum"],
        "independently_calibrated": manifest["independently_calibrated"],
        "flight_authorized": manifest["flight_authorized"],
        "staged_for_bench_build": manifest["staged_for_bench_build"],
        "payload": manifest["files"]["payload"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
