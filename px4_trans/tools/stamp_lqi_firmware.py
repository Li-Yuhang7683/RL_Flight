#!/usr/bin/env python3
"""Bind a built PX4 firmware image to the embedded LQI model manifest."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--airframe-id", required=True)
    parser.add_argument("--firmware", type=Path, required=True)
    parser.add_argument("--manifest", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest_path = (
        args.manifest.expanduser().resolve()
        if args.manifest is not None
        else (
            ROOT
            / "px4_trans/generated"
            / args.airframe_id
            / "lqi_embedded_model.json"
        ).resolve()
    )
    firmware_path = args.firmware.expanduser().resolve()
    if not firmware_path.is_file() or firmware_path.stat().st_size == 0:
        raise ValueError(f"firmware image is missing or empty: {firmware_path}")
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    artifact_type = payload.get("artifact_type")
    if artifact_type == "px4_coaxial_collective_lqi_model":
        actual_airframe = payload.get("airframe_id")
        source = payload.get("source", {})
        source_hashes = {
            "calibration": (
                source["calibration_path"],
                source["calibration_sha256"],
            ),
            "controller_config": (
                source["controller_config"],
                source["controller_sha256"],
            ),
            "simulator_config": (
                source["simulator_config"],
                source["simulator_sha256"],
            ),
            "identification_report": (
                source["identification_report"],
                source["identification_report_sha256"],
            ),
        }
    else:
        actual_airframe = payload.get("airframe", {}).get("airframe_id")
        source_hashes = {
            "identification_report": (
                payload["sources"]["identification_report"],
                payload["sources"]["identification_report_sha256"],
            ),
            "binding_checkpoint": (
                payload["sources"]["binding_checkpoint"],
                payload["sources"]["binding_checkpoint_sha256"],
            ),
        }
    if actual_airframe != args.airframe_id:
        raise ValueError(
            f"manifest belongs to {actual_airframe!r}, expected {args.airframe_id!r}"
        )
    for name, (path_value, expected_sha256) in source_hashes.items():
        source_path = ROOT / path_value
        if sha256(source_path) != expected_sha256:
            raise ValueError(
                f"{name} changed after the LQI model was generated"
            )
    active_header = ROOT / payload["generated"]["active_header"]
    expected_header_sha256 = payload["generated"]["active_header_sha256"]
    actual_header_sha256 = sha256(active_header)
    if actual_header_sha256 != expected_header_sha256:
        raise ValueError(
            "active identified LQI header changed after the model manifest was "
            "generated; regenerate the model before stamping firmware"
        )
    if firmware_path.stat().st_mtime_ns < active_header.stat().st_mtime_ns:
        raise ValueError(
            "firmware is older than the active identified LQI header; rebuild "
            "before stamping it as the airframe firmware artifact"
        )
    payload["firmware_artifact"] = {
        "path": (
            firmware_path.relative_to(ROOT).as_posix()
            if firmware_path.is_relative_to(ROOT)
            else str(firmware_path)
        ),
        "sha256": sha256(firmware_path),
        "size_bytes": firmware_path.stat().st_size,
    }
    manifest_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(json.dumps(payload["firmware_artifact"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
