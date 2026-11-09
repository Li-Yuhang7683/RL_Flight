"""Paired nonlinear validation for the airframe-1 collective/servo LQI."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Mapping

import torch
import yaml


ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True)
class Case:
    name: str
    family: str
    initial_roll_rad: float = 0.0
    initial_pitch_rad: float = 0.0
    target_roll_rad: float = 0.0
    target_pitch_rad: float = 0.0
    yaw_rate_rad_s: float = 0.0
    acceleration_m_s2: float = 0.0


CASES = (
    Case("recover_rp_5", "attitude_recovery", math.radians(5.0), math.radians(-5.0)),
    Case("recover_rp_8", "attitude_recovery", math.radians(-8.0), math.radians(6.0)),
    Case("recover_rp_10", "attitude_recovery", math.radians(10.0), math.radians(4.0)),
    Case("recover_rp_m10", "attitude_recovery", math.radians(-10.0), math.radians(-6.0)),
    Case("track_roll_pos", "attitude_tracking", target_roll_rad=math.radians(5.0)),
    Case("track_roll_neg", "attitude_tracking", target_roll_rad=math.radians(-5.0)),
    Case("track_pitch_pos", "attitude_tracking", target_pitch_rad=math.radians(5.0)),
    Case("track_pitch_neg", "attitude_tracking", target_pitch_rad=math.radians(-5.0)),
    Case("yaw_pos", "yaw_tracking", yaw_rate_rad_s=0.5),
    Case("yaw_neg", "yaw_tracking", yaw_rate_rad_s=-0.5),
    Case("yaw_fast_pos", "yaw_tracking", yaw_rate_rad_s=1.0),
    Case("yaw_fast_neg", "yaw_tracking", yaw_rate_rad_s=-1.0),
    Case("collective_up_1", "collective", acceleration_m_s2=1.0),
    Case("collective_down_1", "collective", acceleration_m_s2=-1.0),
    Case("collective_up_3", "collective", acceleration_m_s2=3.0),
    Case("collective_down_3", "collective", acceleration_m_s2=-3.0),
    Case(
        "coupled_roll_up",
        "collective_attitude",
        target_roll_rad=math.radians(5.0),
        acceleration_m_s2=3.0,
    ),
    Case(
        "coupled_roll_down",
        "collective_attitude",
        target_roll_rad=math.radians(-5.0),
        acceleration_m_s2=-3.0,
    ),
    Case(
        "coupled_pitch_up",
        "collective_attitude",
        target_pitch_rad=math.radians(5.0),
        acceleration_m_s2=3.0,
    ),
    Case(
        "coupled_pitch_down",
        "collective_attitude",
        target_pitch_rad=math.radians(-5.0),
        acceleration_m_s2=-3.0,
    ),
)


def _resolve(path: str | Path) -> Path:
    value = Path(path).expanduser()
    return value if value.is_absolute() else ROOT / value


def _load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return value


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _quaternion(roll: torch.Tensor, pitch: torch.Tensor, yaw: torch.Tensor) -> torch.Tensor:
    cr, sr = torch.cos(roll / 2.0), torch.sin(roll / 2.0)
    cp, sp = torch.cos(pitch / 2.0), torch.sin(pitch / 2.0)
    cy, sy = torch.cos(yaw / 2.0), torch.sin(yaw / 2.0)
    return torch.stack(
        (
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ),
        dim=-1,
    )


def _rotate_body_to_world(
    q_wb: torch.Tensor, vector_b: torch.Tensor
) -> torch.Tensor:
    w, x, y, z = q_wb.unbind(dim=-1)
    # Hamilton body-to-world rotation matrix.
    matrix = torch.stack(
        (
            torch.stack(
                (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
                dim=-1,
            ),
            torch.stack(
                (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
                dim=-1,
            ),
            torch.stack(
                (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
                dim=-1,
            ),
        ),
        dim=-2,
    )
    return torch.bmm(matrix, vector_b.unsqueeze(-1)).squeeze(-1)


def _set_initial_rotor_speed(environment: Any, speed: torch.Tensor) -> None:
    state = environment.state_dict()
    state["truth"]["motor_speed"].copy_(speed)
    state["truth"]["effective_motor_speed"].copy_(speed)
    state["sensors"]["motor_speed"].copy_(speed)
    state["sensor_kernel"]["history"]["motor_speed"].copy_(speed[:, None, :])
    environment.load_state_dict(state)


def _case_scalars(
    name: str,
    values: list[float],
) -> dict[str, float | int | list[float] | None]:
    tensor = torch.tensor(values, dtype=torch.float64)
    return {
        "count": len(values),
        "mean": float(tensor.mean()),
        "median": float(tensor.median()),
        "p95": float(torch.quantile(tensor, 0.95)),
        "max": float(tensor.max()),
    }


def _family_metrics(
    metric_records: Mapping[str, Mapping[str, list[float]]],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for family, metrics in metric_records.items():
        result[family] = {
            name: _case_scalars(name, values)
            for name, values in metrics.items()
            if values
        }
    return result


def _simulate_variant(
    *,
    variant: str,
    controller_config: Mapping[str, Any],
    baseline_mode: str,
    materialized: Any,
    duration_s: float,
    settle_s: float,
    command_change_s: float,
    measurement_mode: str,
    control_hz: int,
) -> dict[str, Any]:
    from flight_controller import (
        ControllerContext,
        ControllerReference,
        ControllerState,
        create_controller,
    )
    from flight_controller.math import (
        lookup,
        quaternion_rotation_error,
        tilt_cosine,
    )
    from flight_controller.plant import LocalPlantModel
    from simenv import SimulationEnvironment

    device = materialized.parameters["body.mass"].device
    dtype = materialized.parameters["body.mass"].dtype
    count = len(CASES)
    initial_state = {
        name: value.clone()
        for name, value in materialized.initial_state.items()
    }
    initial_roll = torch.tensor(
        [case.initial_roll_rad for case in CASES], device=device, dtype=dtype
    )
    initial_pitch = torch.tensor(
        [case.initial_pitch_rad for case in CASES], device=device, dtype=dtype
    )
    zeros = torch.zeros(count, device=device, dtype=dtype)
    initial_state["attitude_q_wb"] = _quaternion(
        initial_roll, initial_pitch, zeros
    )
    initial_state["angular_velocity_b"] = torch.zeros(
        count, 3, device=device, dtype=dtype
    )
    environment = SimulationEnvironment(
        replace(
            materialized,
            initial_state=initial_state,
            sensor_state={
                name: value.clone()
                for name, value in materialized.sensor_state.items()
            },
        ),
        count,
        device,
        dtype,
        logging_enabled=False,
    )

    config = json.loads(json.dumps(controller_config))
    if variant == "legacy_full_allocation":
        config["type"] = "lqr"
        config.setdefault("params", {})[
            "collective_mode"
        ] = baseline_mode
    context = ControllerContext(
        batch_size=count,
        device=device,
        dtype=dtype,
        control_dt=1.0 / control_hz,
        parameters=materialized.parameters,
    )
    controller = create_controller(config, context)
    plant = LocalPlantModel(materialized.parameters)
    _set_initial_rotor_speed(environment, plant.hover_trim().motor_speed)

    target_roll = torch.tensor(
        [case.target_roll_rad for case in CASES], device=device, dtype=dtype
    )
    target_pitch = torch.tensor(
        [case.target_pitch_rad for case in CASES], device=device, dtype=dtype
    )
    target_q = _quaternion(target_roll, target_pitch, zeros)
    level_q = target_q.clone()
    level_q[:, 1:] = 0.0
    level_q[:, 0] = 1.0
    target_yaw_rate = torch.tensor(
        [case.yaw_rate_rad_s for case in CASES], device=device, dtype=dtype
    )
    target_acceleration = torch.tensor(
        [case.acceleration_m_s2 for case in CASES], device=device, dtype=dtype
    )
    attitude_family = torch.tensor(
        [case.family.startswith("attitude") for case in CASES],
        device=device,
        dtype=torch.bool,
    )
    yaw_family = torch.tensor(
        [case.family == "yaw_tracking" for case in CASES],
        device=device,
        dtype=torch.bool,
    )
    collective_family = torch.tensor(
        [
            case.family in {"collective", "collective_attitude"}
            for case in CASES
        ],
        device=device,
        dtype=torch.bool,
    )
    combined_family = torch.tensor(
        [case.family == "collective_attitude" for case in CASES],
        device=device,
        dtype=torch.bool,
    )
    gravity_n = torch.zeros(count, 3, device=device, dtype=dtype)
    gravity_n[:, 2] = 9.80665
    position = torch.zeros(count, 3, device=device, dtype=dtype)
    active = torch.ones(count, device=device, dtype=torch.bool)
    ever_unsafe = torch.zeros(count, device=device, dtype=torch.bool)
    metrics: dict[str, dict[str, list[float]]] = {}
    diagnostic_accumulators = {
        "servo_thrust_leakage_max": torch.zeros(
            count, device=device, dtype=dtype
        ),
        "servo_yaw_leakage_max": torch.zeros(
            count, device=device, dtype=dtype
        ),
        "servo_saturation_fraction": torch.zeros_like(
            zeros, dtype=dtype
        ),
        "servo_nonlinear_thrust_leakage_max": torch.zeros_like(zeros),
        "servo_nonlinear_yaw_leakage_max": torch.zeros_like(zeros),
    }
    all_steps = round(duration_s * control_hz)
    settle_step = round(settle_s * control_hz)
    change_step = round(command_change_s * control_hz)
    final_error = torch.full((count,), torch.inf, device=device, dtype=dtype)
    final_rate = torch.full_like(final_error, torch.inf)
    final_yaw_rate_error = torch.full_like(final_error, torch.inf)
    final_acceleration_error = torch.full_like(final_error, torch.inf)
    acceleration_squared_error = torch.zeros_like(final_error)
    acceleration_count = torch.zeros(count, device=device, dtype=torch.int64)
    yaw_squared_error = torch.zeros_like(final_error)
    yaw_count = torch.zeros_like(acceleration_count)

    try:
        for step in range(all_steps):
            truth = environment.observe("truth").values
            sensor = (
                environment.observe("sensor").values
                if measurement_mode == "simulated_sensors"
                else truth
            )
            angular_velocity = sensor["angular_velocity_b"] if sensor is truth else sensor["gyro"]
            if sensor is truth:
                linear_acceleration_n = truth["linear_acceleration_n"]
                motor_speed = truth["motor_speed"]
            else:
                specific_force_n = _rotate_body_to_world(
                    truth["attitude_q_wb"], sensor["accelerometer"]
                )
                linear_acceleration_n = specific_force_n + gravity_n
                motor_speed = sensor["motor_speed"]
            state = ControllerState(
                position_n=truth["position_n"],
                velocity_n=truth["velocity_n"],
                attitude_q_wb=truth["attitude_q_wb"],
                angular_velocity_b=angular_velocity,
                linear_acceleration_n=linear_acceleration_n,
                motor_speed=motor_speed,
                servo_angle=truth["servo_angle"],
            )

            changed = step >= change_step
            desired_acceleration = torch.where(
                changed & collective_family,
                target_acceleration,
                torch.zeros_like(target_acceleration),
            )
            desired_rate = torch.zeros(count, 3, device=device, dtype=dtype)
            desired_rate[:, 2] = torch.where(
                changed & yaw_family,
                target_yaw_rate,
                torch.zeros_like(target_yaw_rate),
            )
            target_attitude = torch.where(
                (changed & (attitude_family | combined_family))[:, None],
                target_q,
                level_q,
            )
            if variant == "coaxial_collective_servo":
                collective_config = config["params"]["collective"]
                normalized = desired_acceleration / float(
                    collective_config["maximum_acceleration_m_s2"]
                )
                collective = (
                    float(collective_config["pilot_minimum"])
                    + (normalized + 1.0)
                    * 0.5
                    * (
                        float(collective_config["pilot_maximum"])
                        - float(collective_config["pilot_minimum"])
                    )
                ).clamp(0.0, 1.0)
            else:
                desired_thrust = materialized.parameters["body.mass"] * (
                    plant.gravity + desired_acceleration
                )
                _, collective = plant.balanced_motor_target(desired_thrust)
                collective = collective[:, 0]
            reference = ControllerReference(
                target_position_n=position,
                target_velocity_n=position,
                target_attitude_q_wb=target_attitude,
                target_angular_velocity_b=desired_rate,
                collective_command=collective[:, None],
            )
            output = controller.step(state, reference, active)
            result = environment.advance(output.command, active)
            tilt = torch.acos(
                tilt_cosine(truth["attitude_q_wb"]).clamp(-1.0, 1.0)
            )
            unsafe_now = active & (
                (tilt > math.radians(60.0))
                | (
                    torch.linalg.vector_norm(
                        truth["angular_velocity_b"], dim=1
                    )
                    > 10.0
                )
            )
            ever_unsafe |= unsafe_now
            if step >= change_step:
                attitude_error = quaternion_rotation_error(
                    truth["attitude_q_wb"], target_attitude
                )[:, :2].norm(dim=1)
                rate_error = (
                    truth["angular_velocity_b"]
                    - reference.target_angular_velocity_b
                ).norm(dim=1)
                yaw_rate_error = (
                    truth["angular_velocity_b"][:, 2]
                    - reference.target_angular_velocity_b[:, 2]
                )
                measured_vertical_acceleration = -truth[
                    "linear_acceleration_n"
                ][:, 2]
                acceleration_error = desired_acceleration - measured_vertical_acceleration
                if step >= settle_step:
                    yaw_mask = yaw_family & active
                    yaw_squared_error.add_(
                        torch.where(yaw_mask, yaw_rate_error.square(), 0.0)
                    )
                    yaw_count.add_(yaw_mask.to(torch.int64))
                    collective_mask = collective_family & active
                    acceleration_squared_error.add_(
                        torch.where(
                            collective_mask,
                            acceleration_error.square(),
                            0.0,
                        )
                    )
                    acceleration_count.add_(collective_mask.to(torch.int64))
                final_error = torch.where(active, attitude_error, final_error)
                final_rate = torch.where(active, rate_error, final_rate)
                final_yaw_rate_error = torch.where(
                    active, yaw_rate_error.abs(), final_yaw_rate_error
                )
                final_acceleration_error = torch.where(
                    active,
                    acceleration_error.abs(),
                    final_acceleration_error,
                )
                if "controller.achieved_servo_wrench" in output.diagnostics:
                    servo_wrench = output.diagnostics[
                        "controller.achieved_servo_wrench"
                    ]
                    diagnostic_accumulators["servo_thrust_leakage_max"].copy_(
                        torch.maximum(
                            diagnostic_accumulators["servo_thrust_leakage_max"],
                            servo_wrench[:, 0].abs(),
                        )
                    )
                    diagnostic_accumulators["servo_yaw_leakage_max"].copy_(
                        torch.maximum(
                            diagnostic_accumulators["servo_yaw_leakage_max"],
                            servo_wrench[:, 3].abs(),
                        )
                    )
                    diagnostic_accumulators["servo_saturation_fraction"].add_(
                        output.diagnostics[
                            "controller.servo_saturated"
                        ].to(dtype)
                    )
                    commanded_motor_speed = lookup(
                        output.command[:, :2],
                        materialized.parameters[
                            "motors.pwm_to_rpm_table"
                        ],
                    )
                    commanded_servo_angle = lookup(
                        output.command[:, 2:],
                        materialized.parameters["servos.pwm_angle_table"],
                    )
                    commanded_wrench = plant.wrench_from_actuators(
                        commanded_motor_speed, commanded_servo_angle
                    )
                    neutral_servo_wrench = plant.wrench_from_actuators(
                        commanded_motor_speed,
                        torch.zeros_like(commanded_servo_angle),
                    )
                    nonlinear_servo_wrench = (
                        commanded_wrench - neutral_servo_wrench
                    )
                    diagnostic_accumulators[
                        "servo_nonlinear_thrust_leakage_max"
                    ].copy_(
                        torch.maximum(
                            diagnostic_accumulators[
                                "servo_nonlinear_thrust_leakage_max"
                            ],
                            nonlinear_servo_wrench[:, 0].abs(),
                        )
                    )
                    diagnostic_accumulators[
                        "servo_nonlinear_yaw_leakage_max"
                    ].copy_(
                        torch.maximum(
                            diagnostic_accumulators[
                                "servo_nonlinear_yaw_leakage_max"
                            ],
                            nonlinear_servo_wrench[:, 3].abs(),
                        )
                    )
            active &= result.valid
    finally:
        environment.close()

    for index, case in enumerate(CASES):
        family_metrics = metrics.setdefault(case.family, {})
        family_metrics.setdefault("final_attitude_error_deg", []).append(
            math.degrees(float(final_error[index]))
        )
        family_metrics.setdefault("final_rate_error_rad_s", []).append(
            float(final_rate[index])
        )
        if case.family == "yaw_tracking":
            count_i = int(yaw_count[index].item())
            rmse = (
                float(
                    torch.sqrt(
                        yaw_squared_error[index] / max(count_i, 1)
                    )
                )
                if count_i
                else float("inf")
            )
            family_metrics.setdefault("yaw_rate_rmse_rad_s", []).append(rmse)
        if case.family in {"collective", "collective_attitude"}:
            count_i = int(acceleration_count[index].item())
            rmse = (
                float(
                    torch.sqrt(
                        acceleration_squared_error[index]
                        / max(count_i, 1)
                    )
                )
                if count_i
                else float("inf")
            )
            family_metrics.setdefault("vertical_acceleration_rmse_m_s2", []).append(
                rmse
            )

    step_count = max(all_steps - change_step, 1)
    servo_sat = (
        diagnostic_accumulators["servo_saturation_fraction"]
        / step_count
    )
    return {
        "variant": variant,
        "controller_type": controller.controller_type,
        "control_hz": control_hz,
        "safety_rate": float((~ever_unsafe).to(dtype).mean()),
        "cases": {
            case.name: {
                "family": case.family,
                "final_attitude_error_deg": math.degrees(
                    float(final_error[index])
                ),
                "final_rate_error_rad_s": float(final_rate[index]),
                "final_yaw_rate_error_rad_s": float(
                    final_yaw_rate_error[index]
                ),
                "final_vertical_acceleration_error_m_s2": float(
                    final_acceleration_error[index]
                ),
            }
            for index, case in enumerate(CASES)
        },
        "families": _family_metrics(metrics),
        "allocation": {
            "servo_thrust_leakage_abs_max_all_cases_n": float(
                diagnostic_accumulators["servo_thrust_leakage_max"].max()
            ),
            "servo_yaw_leakage_abs_max_all_cases_n_m": float(
                diagnostic_accumulators["servo_yaw_leakage_max"].max()
            ),
            "servo_saturation_fraction_mean": float(servo_sat.mean()),
            "servo_saturation_fraction_max": float(servo_sat.max()),
            "servo_nonlinear_thrust_leakage_abs_max_all_cases_n": float(
                diagnostic_accumulators[
                    "servo_nonlinear_thrust_leakage_max"
                ].max()
            ),
            "servo_nonlinear_yaw_leakage_abs_max_all_cases_n_m": float(
                diagnostic_accumulators[
                    "servo_nonlinear_yaw_leakage_max"
                ].max()
            ),
        }
        if variant == "coaxial_collective_servo"
        else None,
    }


def _provenance(
    simulator_path: Path,
    controller_path: Path,
    controller_config: Mapping[str, Any],
    *,
    allow_missing_identification_report: bool = False,
) -> dict[str, Any]:
    airframe = controller_config.get("airframe", {})
    report_path = _resolve(airframe["identification_report"])
    calibration_path = _resolve(airframe["upper_motor_speed_calibration"])
    report_available = report_path.is_file()
    if not report_available and not allow_missing_identification_report:
        raise FileNotFoundError(
            f"identification report missing: {report_path}; "
            "for offline simulation only, add "
            "--allow-missing-identification-report"
        )

    # Missing reports must never be synthesized from an old model's metadata.
    # Only the controller config identifies the model in this offline-only mode.
    if report_available:
        report = json.loads(report_path.read_text(encoding="utf-8"))
        report_airframe = report["airframe"]
        airframe_id = report_airframe["airframe_id"]
        airframe_number = report_airframe["airframe_number"]
        controller_contract = report_airframe["controller_contract"]
    else:
        airframe_id = airframe["id"]
        airframe_number = None
        controller_contract = None

    header = ROOT / "px4_trans/generated/airframe-1/LqiIdentifiedModel.hpp"
    header_text = header.read_text(encoding="utf-8")
    checksum = airframe["identified_lqi_checksum"]
    return {
        "airframe_id": airframe_id,
        "airframe_number": airframe_number,
        "controller_contract": controller_contract,
        "airframe_identity_source": (
            "identification_report" if report_available else "controller_config_only"
        ),
        "identification_report_available": report_available,
        "identified_lqi_checksum": checksum,
        "checksum_present_in_embedded_header": (
            checksum.removeprefix("0x") + "u" in header_text
        ),
        "simulator_config": str(simulator_path.relative_to(ROOT)),
        "simulator_config_sha256": _sha256(simulator_path),
        "controller_config": str(controller_path.relative_to(ROOT)),
        "controller_config_sha256": _sha256(controller_path),
        "identification_report": str(report_path.relative_to(ROOT)),
        "identification_report_sha256": (
            _sha256(report_path) if report_available else None
        ),
        "upper_motor_speed_calibration": str(
            calibration_path.relative_to(ROOT)
        ),
        "upper_motor_speed_calibration_sha256": _sha256(calibration_path),
        "upper_motor_calibration_domain": airframe[
            "upper_motor_calibration_domain"
        ],
    }


def evaluate(
    config_path: Path,
    output_path: Path,
    *,
    allow_missing_identification_report: bool = False,
) -> dict[str, Any]:
    from flight_controller import load_controller_config
    from simenv.config import load_and_materialize

    config = _load_yaml(config_path)
    simulator_path = _resolve(config["simulator_config"])
    controller_path = _resolve(config["controller_config"])
    baseline_path = _resolve(config["baseline_controller_config"])
    controller_config = load_controller_config(controller_path)
    baseline_config = load_controller_config(baseline_path)
    if controller_config.get("type") != "lqi_coaxial_collective_servo":
        raise ValueError("controller config is not the new isolated type")
    airframe = controller_config.get("airframe")
    if not isinstance(airframe, Mapping):
        raise ValueError("controller config requires an airframe mapping")
    bound_simulator = _resolve(airframe["simulator_config"])
    if bound_simulator.resolve() != simulator_path.resolve():
        raise ValueError(
            "evaluation SimEnv config differs from the controller's "
            "airframe-bound SimEnv config"
        )
    if not _resolve(airframe["identified_lqi_config"]).is_file():
        raise ValueError("identified LQI deployment config is unavailable")
    provenance = _provenance(
        simulator_path,
        controller_path,
        controller_config,
        allow_missing_identification_report=allow_missing_identification_report,
    )
    if provenance["airframe_id"] != "airframe-1":
        raise ValueError("evaluation requires the airframe-1 identification report")
    if not provenance["checksum_present_in_embedded_header"]:
        raise ValueError("identified LQI checksum is not bound to the embedded header")

    dtype_name = str(config.get("dtype", "float32"))
    if dtype_name not in {"float32", "float64"}:
        raise ValueError("evaluation dtype must be float32 or float64")
    dtype = torch.float32 if dtype_name == "float32" else torch.float64
    materialized = load_and_materialize(
        simulator_path,
        len(CASES),
        torch.device("cpu"),
        dtype,
    )
    control_hz = int(materialized.timing.control_hz)
    if control_hz != int(config["control_hz"]):
        raise ValueError(
            "evaluation config control_hz does not match SimEnv"
        )
    variants = {
        "coaxial_collective_servo": _simulate_variant(
            variant="coaxial_collective_servo",
            controller_config=controller_config,
            baseline_mode=str(config["baseline_collective_mode"]),
            materialized=materialized,
            duration_s=float(config["duration_s"]),
            settle_s=float(config["settle_s"]),
            command_change_s=float(config["command_change_s"]),
            measurement_mode=str(config["measurement_mode"]),
            control_hz=control_hz,
        ),
        "legacy_full_allocation": _simulate_variant(
            variant="legacy_full_allocation",
            controller_config=baseline_config,
            baseline_mode=str(config["baseline_collective_mode"]),
            materialized=materialized,
            duration_s=float(config["duration_s"]),
            settle_s=float(config["settle_s"]),
            command_change_s=float(config["command_change_s"]),
            measurement_mode=str(config["measurement_mode"]),
            control_hz=control_hz,
        ),
    }
    new = variants["coaxial_collective_servo"]
    new_recovery = new["families"]["attitude_recovery"][
        "final_attitude_error_deg"
    ]["p95"]
    new_tracking = new["families"]["attitude_tracking"][
        "final_attitude_error_deg"
    ]["p95"]
    new_yaw = new["families"]["yaw_tracking"][
        "yaw_rate_rmse_rad_s"
    ]["mean"]
    new_acceleration = new["families"]["collective"][
        "vertical_acceleration_rmse_m_s2"
    ]["mean"]
    allocation = new["allocation"]
    gates = {
        "all_cases_safe": new["safety_rate"] == 1.0,
        "attitude_recovery_p95_error_below_2_deg": new_recovery < 2.0,
        "attitude_tracking_p95_error_below_2_deg": new_tracking < 2.0,
        "combined_p95_attitude_error_below_2_deg": (
            new["families"]["collective_attitude"][
                "final_attitude_error_deg"
            ]["p95"]
            < 2.0
        ),
        "combined_vertical_acceleration_rmse_below_0_5_m_s2": (
            new["families"]["collective_attitude"][
                "vertical_acceleration_rmse_m_s2"
            ]["mean"]
            < 0.5
        ),
        "yaw_rate_rmse_below_0_2_rad_s": new_yaw < 0.2,
        "vertical_acceleration_rmse_below_0_5_m_s2": (
            new_acceleration < 0.5
        ),
        "servo_thrust_leakage_below_1e_6_n": (
            allocation["servo_thrust_leakage_abs_max_all_cases_n"]
            < 1.0e-6
        ),
        "servo_yaw_leakage_below_1e_5_n_m": (
            allocation["servo_yaw_leakage_abs_max_all_cases_n_m"]
            < 1.0e-5
        ),
        "nonlinear_servo_thrust_leakage_below_5e_3_n": (
            allocation[
                "servo_nonlinear_thrust_leakage_abs_max_all_cases_n"
            ]
            < 5.0e-3
        ),
        "nonlinear_servo_yaw_leakage_below_2e_3_n_m": (
            allocation[
                "servo_nonlinear_yaw_leakage_abs_max_all_cases_n_m"
            ]
            < 2.0e-3
        ),
    }
    report = {
        "schema_version": 1,
        "artifact_type": "coaxial_collective_servo_nonlinear_validation",
        "release_decision": (
            "simulation_validation_only"
            if provenance["identification_report_available"]
            else "simulation_validation_only_missing_provenance"
        ),
        "flight_authorized": False,
        "config": config,
        "provenance": provenance,
        "cases": [case.__dict__ for case in CASES],
        "variants": variants,
        "gates": gates,
        "passed": all(gates.values()),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_suffix(output_path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(output_path)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run paired airframe-1 nonlinear validation"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT
        / "Identification/configs/lqi_coaxial_collective_servo_airframe1.yaml",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT
        / "Identification/runs/lqi_coaxial_collective_servo_airframe1/validation.json",
    )
    parser.add_argument(
        "--allow-missing-identification-report",
        action="store_true",
        help=(
            "Offline simulation only: continue without the archived identification "
            "JSON and mark provenance incomplete. Never enables flight authorization."
        ),
    )
    args = parser.parse_args()
    report = evaluate(
        args.config.resolve(),
        args.output.resolve(),
        allow_missing_identification_report=args.allow_missing_identification_report,
    )
    print(json.dumps(
        {
            "output": str(args.output.resolve()),
            "passed": report["passed"],
            "release_decision": report["release_decision"],
            "identification_report_available": report["provenance"]["identification_report_available"],
            "flight_authorized": report["flight_authorized"],
            "gates": report["gates"],
            "new_allocation": report["variants"][
                "coaxial_collective_servo"
            ]["allocation"],
        },
        indent=2,
    ))
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
