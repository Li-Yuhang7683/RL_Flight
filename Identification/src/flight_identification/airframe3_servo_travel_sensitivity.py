"""Airframe-3 offline servo travel sensitivity: fixed LQI, perturbed plant only.

The airframe-1 manual-v3 controller and its nominal parameters are unchanged
across runs. Only the simulated servo command-to-angle table changes. This is
not PX4 HIL and does not authorize flight.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import replace
from pathlib import Path

import torch

from flight_identification.coaxial_collective_evaluation import (
    ROOT,
    CASES,
    _load_yaml,
    _provenance,
    _resolve,
    _sha256,
    _simulate_variant,
)


def evaluate_servo_travel(config_path: Path, output_path: Path) -> dict:
    from flight_controller import load_controller_config
    from simenv.config import load_and_materialize

    config_path = config_path.resolve()
    config = _load_yaml(config_path)
    simulator_path = _resolve(config["simulator_config"]).resolve()
    controller_path = _resolve(config["controller_config"]).resolve()
    controller = load_controller_config(controller_path)
    if controller.get("type") != "lqi_coaxial_collective_servo":
        raise ValueError("requires coaxial collective servo LQI controller")
    bound_sim = _resolve(controller["airframe"]["simulator_config"]).resolve()
    if bound_sim != simulator_path:
        raise ValueError("controller and evaluator bind different simulator configs")
    if not _resolve(controller["airframe"]["identified_lqi_config"]).is_file():
        raise FileNotFoundError("identified LQI controller config is missing")

    # Explicitly permit the unavailable historical report for this offline-only
    # comparison. Never synthesize a report or claim identified airframe-3 data.
    provenance = _provenance(
        simulator_path,
        controller_path,
        controller,
        allow_missing_identification_report=True,
    )
    if provenance["airframe_id"] != "airframe-1":
        raise ValueError("expected archived airframe-1 controller baseline")
    if not provenance["checksum_present_in_embedded_header"]:
        raise ValueError("archived LQI binding checksum mismatch")

    nominal = load_and_materialize(
        simulator_path, len(CASES), torch.device("cpu"), torch.float32
    )
    table = nominal.parameters["servos.pwm_angle_table"]
    if table.shape[-2:] != (3, 2):
        raise ValueError(f"unexpected servo table shape: {tuple(table.shape)}")
    expected_angle = math.radians(20.0)
    observed_nominal = table[..., -1, 1]
    if not torch.allclose(
        observed_nominal,
        torch.full_like(observed_nominal, 0.35),
        rtol=0,
        atol=1e-5,
    ):
        raise ValueError("source model's positive servo angle no longer equals 0.35 rad")

    results = {}
    for label, angle_rad in (
        ("nominal_20deg", 0.35),
        ("observed_45deg_assumption", math.radians(45.0)),
    ):
        params = dict(nominal.parameters)
        changed_table = table.clone()
        changed_table[..., :, 1] *= angle_rad / 0.35
        params["servos.pwm_angle_table"] = changed_table
        variant_plant = replace(nominal, parameters=params)
        simulated = _simulate_variant(
            variant="coaxial_collective_servo",
            controller_config=controller,
            baseline_mode=str(config["baseline_collective_mode"]),
            materialized=variant_plant,
            controller_parameters=nominal.parameters,
            duration_s=float(config["duration_s"]),
            settle_s=float(config["settle_s"]),
            command_change_s=float(config["command_change_s"]),
            measurement_mode=str(config["measurement_mode"]),
            control_hz=int(config["control_hz"]),
        )
        families = simulated["families"]
        recovery = families["attitude_recovery"]["final_attitude_error_deg"]["p95"]
        tracking = families["attitude_tracking"]["final_attitude_error_deg"]["p95"]
        coupled = families["collective_attitude"]["final_attitude_error_deg"]["p95"]
        metrics = {
            "safety_rate": simulated["safety_rate"],
            "recovery_error_p95_deg": recovery,
            "tracking_error_p95_deg": tracking,
            "coupled_error_p95_deg": coupled,
            "yaw_rate_rmse_rad_s": families["yaw_tracking"]["yaw_rate_rmse_rad_s"]["mean"],
            "vertical_acceleration_rmse_m_s2": families["collective"]["vertical_acceleration_rmse_m_s2"]["mean"],
            "servo_saturation_fraction_mean": simulated["allocation"]["servo_saturation_fraction_mean"],
            "servo_saturation_fraction_max": simulated["allocation"]["servo_saturation_fraction_max"],
            "nonlinear_servo_yaw_leakage_n_m": simulated["allocation"]["servo_nonlinear_yaw_leakage_abs_max_all_cases_n_m"],
        }
        results[label] = {
            "servo_limit_rad": angle_rad,
            "servo_limit_deg": math.degrees(angle_rad),
            "metrics": metrics,
            "screen_passed": (
                metrics["safety_rate"] == 1.0
                and recovery < 2.0
                and tracking < 2.0
                and coupled < 2.0
                and metrics["yaw_rate_rmse_rad_s"] < 0.2
                and metrics["vertical_acceleration_rmse_m_s2"] < 0.5
            ),
            "cases": simulated["cases"],
        }

    report = {
        "schema_version": 1,
        "airframe_under_study": "airframe-3",
        "source_controller_airframe": "airframe-1",
        "controller_gains_and_nominal_parameters_fixed": True,
        "plant_perturbation": "simulated servo command-angle slope, ±20 vs ±45 degrees",
        "servo_45deg_is_visual_estimate": True,
        "does_not_test": [
            "actual servo reversal",
            "actual aerodynamic effectiveness",
            "PX4 firmware HIL",
            "real aircraft stability",
        ],
        "release_decision": "offline_sensitivity_screen_only",
        "flight_authorized": False,
        "identification_report_available": provenance["identification_report_available"],
        "provenance": provenance,
        "source_evaluation_config": str(config_path),
        "source_evaluation_sha256": _sha256(config_path),
        "scenario_count": len(CASES),
        "results": results,
    }
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    temporary.replace(output_path)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compare nominal ±20° and estimated ±45° servo travel with unchanged LQI gains"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "Identification/configs/lqi_coaxial_collective_servo_airframe1_manual_v3.yaml",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp/airframe3_servo_travel_sensitivity.json"),
    )
    args = parser.parse_args()
    report = evaluate_servo_travel(args.config, args.output)
    print(json.dumps({
        "output": str(args.output.resolve()),
        "release_decision": report["release_decision"],
        "flight_authorized": False,
        "controller_gains_and_nominal_parameters_fixed": True,
        "results": {
            name: {
                "screen_passed": result["screen_passed"],
                **result["metrics"],
            }
            for name, result in report["results"].items()
        },
    }, indent=2))


if __name__ == "__main__":
    main()
