from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml


AIRFRAME_SCHEMA_VERSION = 1
ROOT = Path(__file__).resolve().parents[2]
REPOSITORY_ROOT = ROOT.parent
DEFAULT_REGISTRY = ROOT / "configs" / "airframes.yaml"


def file_sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).expanduser().resolve().open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_airframe_registry(
    path: str | Path | None = None,
) -> dict[str, dict[str, Any]]:
    registry_path = (
        DEFAULT_REGISTRY.expanduser().resolve()
        if path is None
        else Path(path).expanduser().resolve()
    )
    payload = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping) or payload.get("schema_version") != 1:
        raise ValueError(f"unsupported airframe registry schema in {registry_path}")
    raw_airframes = payload.get("airframes")
    if not isinstance(raw_airframes, Mapping) or not raw_airframes:
        raise ValueError("airframe registry must define a non-empty airframes map")
    airframes: dict[str, dict[str, Any]] = {}
    numbers: set[int] = set()
    for airframe_id, value in raw_airframes.items():
        if not isinstance(airframe_id, str) or not airframe_id:
            raise ValueError("airframe IDs must be non-empty strings")
        if not isinstance(value, Mapping):
            raise ValueError(f"airframe {airframe_id} must be a mapping")
        record = {"airframe_id": airframe_id, **dict(value)}
        _validate_record(record)
        number = int(record["airframe_number"])
        if number in numbers:
            raise ValueError(f"duplicate airframe_number {number}")
        numbers.add(number)
        airframes[airframe_id] = record
    return airframes


def _validate_record(record: Mapping[str, Any]) -> None:
    required = (
        "airframe_id",
        "airframe_number",
        "display_name",
        "platform_family",
        "controller_contract",
        "status",
    )
    missing = [name for name in required if name not in record]
    if missing:
        raise ValueError(
            f"airframe {record.get('airframe_id', '<unknown>')} is missing fields: "
            f"{missing}"
        )
    number = record["airframe_number"]
    if isinstance(number, bool) or not isinstance(number, int) or number < 1:
        raise ValueError("airframe_number must be a positive integer")
    for name in (
        "airframe_id",
        "display_name",
        "platform_family",
        "controller_contract",
        "status",
    ):
        if not isinstance(record[name], str) or not record[name].strip():
            raise ValueError(f"{name} must be a non-empty string")


def resolve_airframe(
    airframe_id: str,
    registry_path: str | Path | None = None,
) -> dict[str, Any]:
    airframes = load_airframe_registry(registry_path)
    try:
        return airframes[airframe_id]
    except KeyError as error:
        raise ValueError(
            f"unknown airframe_id {airframe_id!r}; available IDs: "
            f"{', '.join(sorted(airframes))}"
        ) from error


def airframe_provenance(record: Mapping[str, Any]) -> dict[str, Any]:
    _validate_record(record)
    return {
        "schema_version": AIRFRAME_SCHEMA_VERSION,
        "airframe_id": str(record["airframe_id"]),
        "airframe_number": int(record["airframe_number"]),
        "display_name": str(record["display_name"]),
        "platform_family": str(record["platform_family"]),
        "controller_contract": str(record["controller_contract"]),
        "status": str(record["status"]),
    }


def validate_airframe_payload(
    payload: Mapping[str, Any],
    expected_airframe_id: str,
    *,
    artifact_name: str,
) -> dict[str, Any]:
    airframe = payload.get("airframe")
    if not isinstance(airframe, Mapping):
        raise ValueError(f"{artifact_name} has no airframe provenance")
    actual_id = airframe.get("airframe_id")
    if actual_id != expected_airframe_id:
        raise ValueError(
            f"{artifact_name} belongs to {actual_id!r}, expected "
            f"{expected_airframe_id!r}"
        )
    return dict(airframe)


def read_bundle_airframe(bundle_path: str | Path) -> dict[str, Any] | None:
    source = Path(bundle_path).expanduser().resolve()
    sidecar = source.parent / "airframe.json"
    sidecar_airframe: dict[str, Any] | None = None
    if sidecar.exists():
        payload = json.loads(sidecar.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping) or not isinstance(
            payload.get("airframe"), Mapping
        ):
            raise ValueError(f"{sidecar} has no airframe provenance")
        sidecar_airframe = dict(payload["airframe"])

    embedded_airframe: dict[str, Any] | None = None
    with np.load(source, allow_pickle=False) as payload:
        if "airframe_json" in payload.files:
            embedded_airframe = json.loads(
                np.asarray(payload["airframe_json"]).reshape(()).item()
            )
            if not isinstance(embedded_airframe, Mapping):
                raise ValueError("bundle airframe_json must contain an object")
            embedded_airframe = dict(embedded_airframe)
        embedded_id = (
            np.asarray(payload["airframe_id"]).reshape(()).item()
            if "airframe_id" in payload.files
            else None
        )
        if (
            embedded_airframe is not None
            and embedded_id is not None
            and embedded_airframe.get("airframe_id") != str(embedded_id)
        ):
            raise ValueError("bundle airframe_id and airframe_json disagree")

    if (
        sidecar_airframe is not None
        and embedded_airframe is not None
        and sidecar_airframe.get("airframe_id")
        != embedded_airframe.get("airframe_id")
    ):
        raise ValueError("bundle embedded and sidecar airframe provenance disagree")
    return sidecar_airframe or embedded_airframe


def write_airframe_sidecar(
    output_directory: str | Path,
    record: Mapping[str, Any],
    *,
    artifact_type: str,
    bindings: Mapping[str, Any] | None = None,
) -> Path:
    destination = Path(output_directory).expanduser().resolve() / "airframe.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": AIRFRAME_SCHEMA_VERSION,
        "artifact_type": artifact_type,
        "airframe": airframe_provenance(record),
        "bindings": {} if bindings is None else dict(bindings),
    }
    destination.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return destination


def _print_record(record: Mapping[str, Any]) -> None:
    print(
        json.dumps(
            {
                "airframe": airframe_provenance(record),
                "log_sources": record.get("log_sources", []),
                "identification": record.get("identification", {}),
                "notes": record.get("notes", []),
            },
            indent=2,
            ensure_ascii=False,
        )
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect the stable airframe registry used by identification"
    )
    parser.add_argument(
        "--registry",
        type=Path,
        default=DEFAULT_REGISTRY,
        help="airframe registry YAML",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("list", help="list registered airframes")
    show = subparsers.add_parser("show", help="show one airframe")
    show.add_argument("airframe_id")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    registry_path = Path(args.registry).expanduser().resolve()
    if args.command == "list":
        for airframe_id, record in sorted(
            load_airframe_registry(registry_path).items()
        ):
            identity = airframe_provenance(record)
            print(
                f"{airframe_id}: number={identity['airframe_number']}, "
                f"name={identity['display_name']}, status={identity['status']}"
            )
    else:
        _print_record(resolve_airframe(args.airframe_id, registry_path))


if __name__ == "__main__":
    main()
