"""Offline airframe-3 robustness screen: measured mass, uncertain inertia/servos.

The archived airframe-1 manual-v3 LQI controller model/gains stay fixed.
Only simulated plant parameters are varied. This is neither PX4 HIL nor
flight authorization; uncertainty ranges are deliberate stress assumptions.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path

import torch

from flight_identification.coaxial_collective_evaluation import (
    CASES,
    ROOT,
    _load_yaml,
    _provenance,
    _resolve,
    _sha256,
    _simulate_variant,
)


@dataclass(frozen=True)
class Scenario:
    name: str
    mass_kg: float
    inertia_scale: float
    servo_travel_deg: float


MEASURED_MASS_KG = 0.655
SCENARIOS = (
    Scenario("measured_655g_servo_20deg", MEASURED_MASS_KG, 1.0, 20.0),
    Scenario("measured_655g_servo_45deg", MEASURED_MASS_KG, 1.0, 45.0),
    Scenario("mass_minus_5pct", MEASURED_MASS_KG * 0.95, 1.0, 45.0),
    Scenario("mass_plus_5pct", MEASURED_MASS_KG * 1.05, 1.0, 45.0),
    Scenario("inertia_minus_30pct", MEASURED_MASS_KG, 0.70, 45.0),
    Scenario("inertia_plus_30pct", MEASURED_MASS_KG, 1.30, 45.0),
    Scenario("servo_36deg", MEASURED_MASS_KG, 1.0, 36.0),
    Scenario("servo_54deg", MEASURED_MASS_KG, 1.0, 54.0),
    Scenario("combined_heavy_high_inertia_low_servo", MEASURED_MASS_KG * 1.05, 1.30, 36.0),
    Scenario("combined_light_low_inertia_high_servo", MEASURED_MASS_KG * 0.95, 0.70, 54.0),
)


def validate_scenario(nominal: object, scenario: Scenario) -> dict[str, torch.Tensor]:
    parameters = dict(nominal.parameters)
    current_mass = parameters["body.mass"]
    inertia = parameters["body.inertia_diagonal_b"]
    original_servo_table = parameters["servos.pwm_angle_table"]
    if (
        current_mass.ndim < 1
        or inertia.shape[-1] != 3
        or original_servo_table.shape[-2:] != (3, 2)
        or not torch.allclose(
            original_servo_table[..., -1, 1],
            torch.full_like(original_servo_table[..., -1, 1], 0.35),
            atol=1e-5,
            rtol=0.0,
        )
    ):
        raise ValueError("unexpected SimEnv shape or servo baseline: refusing stress sweep")
    parameters["body.mass"] = torch.full_like(current_mass, scenario.mass_kg)
    parameters["body.inertia_diagonal_b"] = inertia.clone() * scenario.inertia_scale
    # Scale only the servo command-to-angle response, not the controller model.
    table = original_servo_table.clone()
    table[..., :, 1] *= math.radians(scenario.servo_travel_deg) / 0.35
    parameters["servos.pwm_angle_table"] = table
    return parameters


def evaluate(config_path: Path, output_path: Path) -> dict:
    from flight_controller import load_controller_config
    from simenv.config import load_and_materialize

    config_path = config_path.resolve()
    cfg = _load_yaml(config_path)
    sim_path = _resolve(cfg["simulator_config"]).resolve()
    controller_path = _resolve(cfg["controller_config"]).resolve()
    controller = load_controller_config(controller_path)
    if controller.get("type") != "lqi_coaxial_collective_servo":
        raise ValueError("requires coaxial collective servo LQI")
    airframe = controller["airframe"]
    if _resolve(airframe["simulator_config"]).resolve() != sim_path:
        raise ValueError("bound SimEnv does not match evaluator config")
    if not _resolve(airframe["identified_lqi_config"]).is_file():
        raise FileNotFoundError("archived LQI controller config unavailable")

    provenance = _provenance(
        sim_path, controller_path, controller,
        allow_missing_identification_report=True,
    )
    if provenance["airframe_id"] != "airframe-1":
        raise ValueError("expected source airframe-1 controller")
    if not provenance["checksum_present_in_embedded_header"]:
        raise ValueError("source LQI checksum mismatch")

    dtype_name = str(cfg.get("dtype", "float32"))
    if dtype_name not in ("float32", "float64"):
        raise ValueError("unsupported dtype")
    dtype = torch.float32 if dtype_name == "float32" else torch.float64
    nominal = load_and_materialize(
        sim_path, len(CASES), torch.device("cpu"), dtype
    )
    if int(nominal.timing.control_hz) != int(cfg["control_hz"]):
        raise ValueError("controller rate mismatch")
    nominal_mass_kg = float(nominal.parameters["body.mass"].flatten()[0])
    results = {}
    for index, scenario in enumerate(SCENARIOS, 1):
        perturbed = replace(
            nominal, parameters=validate_scenario(nominal, scenario)
        )
        # The controller's model and LQI gains remain the archived nominal.
        # The simulator gets the perturbed mass, inertia and servo response.
        data = _simulate_variant(
            variant="coaxial_collective_servo",
            controller_config=controller,
            baseline_mode=str(cfg["baseline_collective_mode"]),
            materialized=perturbed,
            controller_parameters=nominal.parameters,
            duration_s=float(cfg["duration_s"]),
            settle_s=float(cfg["settle_s"]),
            command_change_s=float(cfg["command_change_s"]),
            measurement_mode=str(cfg["measurement_mode"]),
            control_hz=int(cfg["control_hz"]),
        )
        families = data["families"]
        allocation = data["allocation"]
        metrics = {
            "safety_rate": data["safety_rate"],
            "recovery_error_p95_deg": families["attitude_recovery"]["final_attitude_error_deg"]["p95"],
            "tracking_error_p95_deg": families["attitude_tracking"]["final_attitude_error_deg"]["p95"],
            "coupled_error_p95_deg": families["collective_attitude"]["final_attitude_error_deg"]["p95"],
            "yaw_rate_rmse_rad_s": families["yaw_tracking"]["yaw_rate_rmse_rad_s"]["mean"],
            "vertical_acceleration_rmse_m_s2": families["collective"]["vertical_acceleration_rmse_m_s2"]["mean"],
            "servo_saturation_fraction_mean": allocation["servo_saturation_fraction_mean"],
            "servo_saturation_fraction_max": allocation["servo_saturation_fraction_max"],
            "nonlinear_servo_yaw_leakage_n_m": allocation["servo_nonlinear_yaw_leakage_abs_max_all_cases_n_m"],
            "nonlinear_servo_thrust_leakage_n": allocation["servo_nonlinear_thrust_leakage_abs_max_all_cases_n"],
        }
        gates = {
            "all_cases_safe": metrics["safety_rate"] == 1.0,
            "recovery_p95_lt_2deg": metrics["recovery_error_p95_deg"] < 2.0,
            "tracking_p95_lt_2deg": metrics["tracking_error_p95_deg"] < 2.0,
            "coupled_p95_lt_2deg": metrics["coupled_error_p95_deg"] < 2.0,
            "yaw_rmse_lt_0_2rad_s": metrics["yaw_rate_rmse_rad_s"] < 0.2,
            "acceleration_rmse_lt_0_5m_s2": metrics["vertical_acceleration_rmse_m_s2"] < 0.5,
            "servo_yaw_leakage_lt_0_002Nm": metrics["nonlinear_servo_yaw_leakage_n_m"] < 0.002,
            "servo_thrust_leakage_lt_0_005N": metrics["nonlinear_servo_thrust_leakage_n"] < 0.005,
        }
        results[scenario.name] = {
            "mass_kg": scenario.mass_kg,
            "inertia_scale": scenario.inertia_scale,
            "servo_travel_deg": scenario.servo_travel_deg,
            "screen_passed": all(gates.values()),
            "gates": gates,
            "metrics": metrics,
            "cases": data["cases"],
        }
        print(
            f"[{index}/{len(SCENARIOS)}] {scenario.name} "
            f"screen_passed={all(gates.values())} "
            f"safety={metrics['safety_rate']:.2%} "
            f"recovery_p95={metrics['recovery_error_p95_deg']:.3f}deg",
            flush=True,
        )

    report = {
        "schema_version": 1,
        "airframe_under_study": "airframe-3",
        "source_controller_airframe": "airframe-1",
        "measured_airframe3_total_mass_kg": MEASURED_MASS_KG,
        "archived_nominal_mass_kg": nominal_mass_kg,
        "uncertainty_bounds_are_stress_assumptions_not_measurements": True,
        "controller_gains_and_nominal_parameters_fixed": True,
        "simulation_hover_initialization": "ideal_plant_hover_trim",
        "scenario_count": len(SCENARIOS),
        "cases_per_scenario": len(CASES),
        "release_decision": "offline_robustness_screen_only",
        "flight_authorized": False,
        "does_not_test": [
            "real airframe-3 motor thrust/torque and effectiveness",
            "true aerodynamic grid moment sign and strength",
            "servo reversal or asymmetric physical limits",
            "PX4 HIL or actuator signal path",
            "failure modes, worst-case multi-parameter uncertainty",
            "real aircraft flight stability",
        ],
        "identification_report_available": provenance["identification_report_available"],
        "provenance": provenance,
        "source_evaluation_config": str(config_path),
        "source_evaluation_sha256": _sha256(config_path),
        "results": results,
    }
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    temporary_path.replace(output_path)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", type=Path,
        default=ROOT / "Identification/configs/lqi_coaxial_collective_servo_airframe1_manual_v3.yaml",
    )
    parser.add_argument(
        "--output", type=Path,
        default=Path("/tmp/airframe3_655g_robustness.json"),
    )
    args = parser.parse_args()
    report = evaluate(args.config, args.output)
    print(json.dumps({
        "output": str(args.output.resolve()),
        "release_decision": report["release_decision"],
        "flight_authorized": False,
        "measured_airframe3_total_mass_kg": report["measured_airframe3_total_mass_kg"],
        "scenario_count": report["scenario_count"],
        "passed_scenarios": sum(
            s["screen_passed"] for s in report["results"].values()
        ),
        "failed_scenarios": [
            name for name, s in report["results"].items()
            if not s["screen_passed"]
        ],
    }, indent=2))


if __name__ == "__main__":
    main()
