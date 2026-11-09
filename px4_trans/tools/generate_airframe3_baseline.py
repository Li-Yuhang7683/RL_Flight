#!/usr/bin/env python3
"""Copy the archived airframe-1 manual-v3 LQI model for airframe-3 BENCH use.

This copies an existing, SHA-256 verified model payload rather than
re-solving an LQI/DARE (which can alter float32 bytes across SciPy builds).
Only the embedded airframe identity changes; none of the numeric model
coefficients or their model checksum changes.

No independent airframe-3 identification, airworthiness, or actuator-output
authorization is implied. Default: archive only, no PX4 active-file changes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "Identification/configs/airframe3_baseline_from_airframe1.yaml"
SOURCE_ARCHIVE = ROOT / "px4_trans/generated/airframe-1/manual-v3"
ARCHIVED_FILES = (
    ("payload", "LqiCoaxialCollectivePayload.hpp", "candidate_payload_sha256"),
    ("golden_header", "lqi_coaxial_collective_golden.hpp", "candidate_golden_header_sha256"),
    ("golden_json", "lqi_coaxial_collective_golden.json", "candidate_golden_json_sha256"),
)
STAGE_PATHS = {
    "payload": ROOT / "px4_trans/px4/src/modules/nn_control/LqiCoaxialCollectivePayload.hpp",
    "golden_header": ROOT / "px4_trans/tests/lqi_coaxial_collective_golden.hpp",
    "golden_json": ROOT / "px4_trans/tests/lqi_coaxial_collective_golden.json",
}


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_verified_source(expected_checksum: str) -> tuple[dict[str, Any], dict[str, Path]]:
    """Verify exact archived bytes against the historical publication manifest."""
    manifest_path = SOURCE_ARCHIVE / "lqi_coaxial_collective_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("airframe_id") != "airframe-1" or manifest.get("airframe_number") != 1:
        raise ValueError("archived model does not belong to airframe-1")
    if manifest.get("model_checksum", "").lower() != expected_checksum.lower():
        raise ValueError(
            "archived airframe-1 model checksum does not match baseline pin: "
            f"archive={manifest.get('model_checksum')}, expected={expected_checksum}"
        )
    source_files: dict[str, Path] = {}
    for role, filename, hash_field in ARCHIVED_FILES:
        source_file = SOURCE_ARCHIVE / filename
        expected_sha = manifest["generated"][hash_field]
        actual_sha = file_sha256(source_file)
        if actual_sha != expected_sha:
            raise ValueError(
                f"archived {filename} SHA-256 mismatch: "
                f"actual={actual_sha}, expected={expected_sha}"
            )
        source_files[role] = source_file

    header = source_files["payload"].read_text(encoding="utf-8")
    if header.count("static constexpr uint32_t kAirframeNumber{1u};") != 1:
        raise ValueError("unexpected archived embedded airframe number")
    if header.count('static constexpr char kAirframeId[]{"airframe-1"};') != 1:
        raise ValueError("unexpected archived embedded airframe ID")
    checksum_match = re.search(r"\b0x([0-9a-fA-F]{8})u,\s*\};", header)
    if not checksum_match or "0x" + checksum_match.group(1).lower() != expected_checksum.lower():
        raise ValueError("archived C++ model checksum does not match archive manifest")
    return manifest, source_files


def rebind_identity_only(header: str) -> str:
    """Change airframe constants only; preserve all numeric control coefficients."""
    number = "static constexpr uint32_t kAirframeNumber{1u};"
    identity = 'static constexpr char kAirframeId[]{"airframe-1"};'
    if header.count(number) != 1 or header.count(identity) != 1:
        raise ValueError("expected airframe-1 identity constants missing or duplicated")
    return (
        "// AIRFRAME-3 INHIBITED BENCH BASELINE. "
        "Inherited airframe-1 manual-v3 model; NOT independently identified.\n"
        + header.replace(
            number, "static constexpr uint32_t kAirframeNumber{3u};"
        ).replace(
            identity, 'static constexpr char kAirframeId[]{"airframe-3"};'
        )
    )


def prepare(config_path: Path, *, stage_for_bench_build: bool = False) -> dict[str, Any]:
    from flight_identification.airframe import resolve_airframe

    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if config.get("schema_version") != 1:
        raise ValueError("unsupported baseline schema")
    if (config.get("target_airframe_id"), config.get("source_airframe_id")) != (
        "airframe-3", "airframe-1"
    ):
        raise ValueError("only airframe-1 to airframe-3 baseline is supported")
    if config.get("flight_authorized") is not False:
        raise ValueError("baseline must stay flight_authorized=false")
    if config.get("independently_calibrated") is not False:
        raise ValueError("baseline cannot claim independent airframe-3 calibration")
    if config.get("purpose") != (
        "offline_simulation_and_inhibited_bench_firmware_only"
    ):
        raise ValueError("baseline purpose must remain bench-only")
    if config.get("parameter_provenance") != (
        "assumed_identical_to_airframe1_manual_v3"
    ):
        raise ValueError("invalid source parameter provenance")
    approved_source_config = (
        ROOT / "px4_trans/configs/lqi_coaxial_collective_airframe1_manual_v3.yaml"
    ).resolve()
    source_config = (ROOT / config["source_deployment_config"]).resolve()
    if source_config != approved_source_config:
        raise ValueError("source config must be airframe-1 manual-v3")
    source_deployment = yaml.safe_load(source_config.read_text(encoding="utf-8"))
    if source_deployment.get("airframe_id") != "airframe-1":
        raise ValueError("source deployment config has incorrect airframe identity")

    airframe = resolve_airframe(
        "airframe-3", ROOT / config["airframe_registry"]
    )
    if airframe["airframe_number"] != 3 or airframe["controller_contract"] != (
        "pilot_acceleration_dual_motor_common_differential_servo_roll_pitch"
    ):
        raise ValueError("airframe-3 registration or controller contract mismatch")
    if airframe["identification"].get("flight_authorized") is not False:
        raise ValueError("airframe-3 cannot be flight-authorized by this script")

    expected_checksum = str(config["expected_source_model_checksum"])
    source_manifest, sources = load_verified_source(expected_checksum)
    output_dir = (ROOT / config["output_dir"]).resolve()
    approved_output = (ROOT / "px4_trans/generated/airframe-3").resolve()
    if not output_dir.is_relative_to(approved_output):
        raise ValueError("generated candidate must remain in airframe-3 archive")

    output_dir.mkdir(parents=True, exist_ok=True)
    destination = {role: output_dir / src.name for role, src in sources.items()}
    rebound = rebind_identity_only(
        sources["payload"].read_text(encoding="utf-8")
    )
    destination["payload"].write_text(rebound, encoding="utf-8")
    for role in ("golden_header", "golden_json"):
        shutil.copyfile(sources[role], destination[role])

    report_name = source_manifest["source"].get("identification_report")
    report_path = ROOT / report_name if report_name else None
    report_available = bool(report_path and report_path.is_file())
    archived_report_sha = source_manifest["source"].get(
        "identification_report_sha256"
    )
    if report_available and file_sha256(report_path) != archived_report_sha:
        raise ValueError("available historical identification report SHA-256 mismatch")

    manifest: dict[str, Any] = {
        "schema_version": 1,
        "artifact_type": "assumed_parameter_coaxial_lqi_bench_candidate",
        "target_airframe_id": "airframe-3",
        "target_airframe_number": 3,
        "source_airframe_id": "airframe-1",
        "model_checksum": source_manifest["model_checksum"],
        "parameter_provenance": config["parameter_provenance"],
        "independently_calibrated": False,
        "flight_authorized": False,
        "actuator_output_authorized": False,
        "purpose": config["purpose"],
        "source_archive_manifest": str(
            (SOURCE_ARCHIVE / "lqi_coaxial_collective_manifest.json").relative_to(ROOT)
        ),
        "source_archive_manifest_sha256": file_sha256(
            SOURCE_ARCHIVE / "lqi_coaxial_collective_manifest.json"
        ),
        "source_identification_report": report_name,
        "source_identification_report_expected_sha256": archived_report_sha,
        "source_identification_report_available": report_available,
        "source_model_sha256": {
            role: file_sha256(src) for role, src in sources.items()
        },
        "files": {
            role: str(src.relative_to(ROOT)) for role, src in destination.items()
        },
        "generated_sha256": {
            role: file_sha256(src) for role, src in destination.items()
        },
        "staged_for_bench_build": False,
        "notes": [
            "Airframe-3 inherits the exact archived airframe-1 manual-v3 model bytes.",
            "Only embedded airframe number and ID change; golden vectors remain unchanged.",
            "No new airframe-3 physical measurements or flight authorization.",
        ],
    }

    if stage_for_bench_build:
        param_path = ROOT / "px4_trans/px4/src/modules/nn_control/nn_control_params.c"
        if not param_path.is_file() or (
            "PARAM_DEFINE_INT32(NN_LQI_OUTPUT_EN, 0);" not in param_path.read_text(
                encoding="utf-8"
            )
        ):
            raise ValueError(
                "PX4 source not initialized or actuator-output default is not inhibited"
            )
        for role, active in STAGE_PATHS.items():
            if not active.is_file():
                raise FileNotFoundError(
                    f"active PX4 file not present: {active}; initialize submodule first"
                )
        backups: dict[str, str] = {}
        for role, active in STAGE_PATHS.items():
            backup = output_dir / ("previous_active_" + active.name)
            shutil.copyfile(active, backup)
            backups[role] = str(backup.relative_to(ROOT))
            shutil.copyfile(destination[role], active)
        manifest["staged_for_bench_build"] = True
        manifest["previous_active_backups"] = backups

    manifest_path = output_dir / "lqi_coaxial_collective_manifest.json"
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
        help="only stage the inhibited bench candidate for a local PX4 build",
    )
    args = parser.parse_args()
    result = prepare(
        args.config.resolve(), stage_for_bench_build=args.stage_for_bench_build
    )
    print(json.dumps({
        "target_airframe_id": result["target_airframe_id"],
        "model_checksum": result["model_checksum"],
        "source_identification_report_available":
            result["source_identification_report_available"],
        "independently_calibrated": result["independently_calibrated"],
        "flight_authorized": result["flight_authorized"],
        "staged_for_bench_build": result["staged_for_bench_build"],
        "payload": result["files"]["payload"],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
