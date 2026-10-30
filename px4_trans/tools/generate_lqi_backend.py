#!/usr/bin/env python3
"""Generate the PX4 nominal-LQI model payload and host golden vectors.

The generated header contains data only.  The reusable C++ algorithm lives in
LqiControllerCore.hpp and consumes the stable descriptor in
LqiControllerModel.hpp.

Run from the repository root:
    PYTHONPATH=Identification/src:Controller/src:SimEnv/src \
    python px4_trans/tools/generate_lqi_backend.py

The default deployment manifest is bound to `airframe-1`. Airframe-specific
manifests must carry matching `airframe_id` and identification report fields.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Mapping

import numpy as np
import torch
import yaml

from flight_identification.airframe import (
    DEFAULT_REGISTRY,
    airframe_provenance,
    file_sha256,
    resolve_airframe,
    validate_airframe_payload,
)


ROOT = Path(__file__).resolve().parents[2]
MODEL_HEADER = ROOT / "px4_trans/px4/src/modules/nn_control/LqiNominalModel.hpp"
GOLDEN_HEADER = ROOT / "px4_trans/tests/lqi_golden.hpp"
GOLDEN_JSON = ROOT / "px4_trans/tests/lqi_golden.json"
COMPOSITE_MODEL_HEADER = ROOT / "px4_trans/px4/src/modules/nn_control/LqiIdentifiedModel.hpp"
COMPOSITE_GOLDEN_HEADER = ROOT / "px4_trans/tests/lqi_identified_golden.hpp"
COMPOSITE_GOLDEN_JSON = ROOT / "px4_trans/tests/lqi_identified_golden.json"
DEPLOYMENT_CONFIG = ROOT / "px4_trans/configs/lqi_airframe_1.yaml"
EMBEDDED_MANIFEST_ROOT = ROOT / "px4_trans/generated"
DT_S = 0.002
RPM_TO_RAD_S = 2.0 * math.pi / 60.0


def f32(value: float) -> np.float32:
    return np.float32(value)


def load_model(
    deployment_config: Path,
) -> dict[str, np.ndarray | np.float32]:
    from flight_controller import ControllerContext, create_controller, load_controller_config
    from flight_identification.control_evaluation import _nominal_actuator_model
    from simenv.config import load_and_materialize

    deployment = yaml.safe_load(deployment_config.read_text(encoding="utf-8"))
    if deployment.get("schema_version") not in (1, 2):
        raise ValueError("unsupported LQI deployment schema")
    simulator_config = ROOT / deployment["simulator_config"]
    controller_config = ROOT / deployment["controller_config"]
    materialized = load_and_materialize(
        simulator_config, 1, torch.device("cpu"), torch.float64
    )
    config = load_controller_config(controller_config)
    controller = create_controller(
        config,
        ControllerContext(
            batch_size=1,
            device=torch.device("cpu"),
            dtype=torch.float64,
            control_dt=DT_S,
            parameters=materialized.parameters,
        ),
    )
    gain = controller._lqr_gain
    if gain is None or tuple(gain.shape) != (4, 13):
        raise RuntimeError(f"nominal controller must provide a 4x13 gain, got {None if gain is None else tuple(gain.shape)}")
    _, slopes, time_constants = _nominal_actuator_model(simulator_config)
    decay = np.exp(-DT_S / time_constants).astype(np.float32)
    sticks = deployment["manual_reference"]
    command_tau = np.asarray(
        [
            sticks["roll"]["time_constant_s"],
            sticks["pitch"]["time_constant_s"],
            sticks["yaw"]["time_constant_s"],
        ],
        dtype=np.float32,
    )
    attitude = config["params"]["pid"]["attitude"]
    trim = controller.trim
    return {
        "gain": gain.detach().cpu().numpy().astype(np.float32),
        "decay": decay,
        "slope_eff": (slopes.astype(np.float32) * (1.0 - decay)).astype(np.float32),
        "integral_limit": np.asarray(attitude["integral_limit"], dtype=np.float32),
        "motor_table": materialized.parameters["motors.pwm_to_rpm_table"][0, 0].detach().cpu().numpy().astype(np.float32),
        "dt": f32(DT_S),
        "upper_trim": f32(trim.command[0, 0].item()),
        "lower_trim": f32(trim.command[0, 1].item()),
        "upper_trim_rad_s": f32(trim.motor_speed[0, 0].item()),
        "lower_trim_rad_s": f32(trim.motor_speed[0, 1].item()),
        "rpm_to_rad_s": f32(RPM_TO_RAD_S),
        "max_roll_angle": f32(sticks["roll"]["limit_rad"]),
        "max_pitch_angle": f32(sticks["pitch"]["limit_rad"]),
        "max_yaw_rate": f32(sticks["yaw"]["limit_rad_s"]),
        "manual_command_decay": np.exp(-DT_S / command_tau).astype(np.float32),
    }


def quaternion_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    tw, tx, ty, tz = target
    cw, cx, cy, cz = current
    error = np.asarray(
        [
            tw * cw + tx * cx + ty * cy + tz * cz,
            tw * cx - tx * cw - ty * cz + tz * cy,
            tw * cy + tx * cz - ty * cw - tz * cx,
            tw * cz - tx * cy + ty * cx - tz * cw,
        ],
        dtype=np.float32,
    )
    if error[0] < 0:
        error = -error
    norm = np.sqrt(np.sum(error[1:] * error[1:], dtype=np.float32), dtype=np.float32)
    angle = f32(2.0) * np.arctan2(norm, np.maximum(error[0], f32(1e-12)))
    return (error[1:] * angle / np.maximum(norm, f32(1e-12))).astype(np.float32)


def lqi_step_float32(
    current_q: np.ndarray,
    target_q: np.ndarray,
    current_rate: np.ndarray,
    target_rate: np.ndarray,
    collective: float,
    motor_rpm: np.ndarray,
    motor_rpm_valid: bool,
    persistent: np.ndarray,
    model: dict[str, np.ndarray | np.float32],
) -> tuple[np.ndarray, np.ndarray, np.float32]:
    error = quaternion_error(current_q, target_q)
    rate_error = (current_rate - target_rate).astype(np.float32)
    collective_f = np.clip(f32(collective), f32(0.0), f32(1.0)).astype(np.float32)
    if collective_f <= f32(0.0):
        # Idle throttle: no actuator output and persistent state stays frozen.
        # Must match LqiControllerCore::stepFromState.
        return (
            np.zeros(4, dtype=np.float32),
            persistent.copy(),
            collective_f,
        )
    target_speed = f32(np.interp(float(collective_f), model["motor_table"][:, 0], model["motor_table"][:, 1]))
    target_dev = target_speed - np.asarray(
        [model["upper_trim_rad_s"], model["lower_trim_rad_s"]], dtype=np.float32
    )
    if motor_rpm_valid:
        motor_error = np.abs(motor_rpm).astype(np.float32) * model["rpm_to_rad_s"] - np.asarray(
            [model["upper_trim_rad_s"], model["lower_trim_rad_s"]], dtype=np.float32
        ) - target_dev
    else:
        motor_error = persistent[:2] - target_dev
    state = np.concatenate(
        (error[:2], rate_error, motor_error, persistent[2:5], persistent[5:8])
    ).astype(np.float32)
    delta = np.empty(4, dtype=np.float32)
    for row in range(4):
        accumulator = f32(0.0)
        for col in range(13):
            accumulator = f32(accumulator + f32(model["gain"][row, col]) * state[col])
        delta[row] = -accumulator
    raw = np.concatenate((collective_f + delta[:1], delta[1:])).astype(np.float32)
    # Keep the lower-rotor opening within 20 percentage points of the pilot
    # upper-rotor command, additionally bounded to [0, 1]. Must match
    # LqiControllerCore::stepFromState.
    lower_min = np.maximum(collective_f - f32(0.2), f32(0.0)).astype(np.float32)
    lower_max = np.minimum(collective_f + f32(0.2), f32(1.0)).astype(np.float32)
    output = np.concatenate(
        (np.clip(raw[:1], lower_min, lower_max), np.clip(raw[1:], f32(-1.0), f32(1.0)))
    ).astype(np.float32)
    saturated = bool(
        np.any(raw[:1] < lower_min)
        or np.any(raw[:1] > lower_max)
        or np.any(raw[1:] < -1.0)
        or np.any(raw[1:] > 1.0)
    )
    integral_error = np.asarray([error[0], error[1], rate_error[2]], dtype=np.float32)
    candidate = np.clip(
        persistent[5:8] + integral_error * model["dt"],
        -model["integral_limit"],
        model["integral_limit"],
    ).astype(np.float32)
    command_dev = np.concatenate(([collective_f - model["upper_trim"]], output - np.asarray(
        [model["lower_trim"], 0.0, 0.0, 0.0], dtype=np.float32
    ))).astype(np.float32)
    actuator_next = (model["decay"] * persistent[:5] + model["slope_eff"] * command_dev).astype(np.float32)
    if motor_rpm_valid:
        actuator_next[:2] = np.abs(motor_rpm).astype(np.float32) * model["rpm_to_rad_s"] - np.asarray(
            [model["upper_trim_rad_s"], model["lower_trim_rad_s"]], dtype=np.float32
        )
    next_persistent = np.concatenate((actuator_next, persistent[5:8] if saturated else candidate)).astype(np.float32)
    return output, next_persistent, collective_f


def golden_vectors(model: dict[str, np.ndarray | np.float32]) -> list[dict[str, object]]:
    cases = [
        ([1, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0], [0, 0, 0], 0.5, [0, 0], False, [0] * 8),
        ([0.9998, 0.01, 0, 0], [1, 0, 0, 0], [0.1, -0.2, 0.05], [0, 0, 0.3], 0.75, [11000, -11500], True, [0] * 8),
        ([0.9, 0.2, -0.3, 0.1], [0.98, -0.1, 0.12, 0.02], [-1, 0.8, 0.6], [0, 0, -0.5], 0.2, [9800, 10200], True, [100, -50, 0.05, -0.02, 0.03, 0.1, -0.2, 0.3]),
        ([-0.9998, -0.01, 0, 0], [1, 0, 0, 0], [0.3, 0.4, 0.2], [0, 0, 1.2], 1.0, [0, 0], False, [500, 400, 0.3, 0.25, 0.2, 0.2, 0.2, 0.4]),
        ([0.8, 0.4, 0.4, 0.2], [0.9239, 0, 0, 0.3827], [3, -3, 2], [0, 0, 0], 0.0, [13500, 9000], True, [-300, 200, -0.3, 0, 0.1, -0.25, 0.15, -0.45]),
    ]
    vectors = []
    for current_q, target_q, current_rate, target_rate, collective, rpm, valid, persistent in cases:
        arrays = [np.asarray(value, dtype=np.float32) for value in (current_q, target_q, current_rate, target_rate, rpm, persistent)]
        output, next_state, upper = lqi_step_float32(*arrays[:4], collective, arrays[4], valid, arrays[5], model)
        vectors.append({
            "attitude_q_wb": arrays[0].tolist(), "target_attitude_q_wb": arrays[1].tolist(),
            "angular_velocity_b": arrays[2].tolist(), "target_angular_velocity_b": arrays[3].tolist(),
            "collective_base": float(collective), "motor_rpm": arrays[4].tolist(), "motor_rpm_valid": valid,
            "persistent": arrays[5].tolist(), "expected_output": output.tolist(),
            "expected_upper": float(upper),
            "expected_next_persistent": next_state.tolist(),
        })
    return vectors


def fnv1a32(data: bytes) -> int:
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


def checksum(model: dict[str, np.ndarray | np.float32]) -> int:
    names = ("gain", "decay", "slope_eff", "integral_limit", "motor_table", "dt", "upper_trim", "lower_trim", "upper_trim_rad_s", "lower_trim_rad_s", "rpm_to_rad_s", "max_roll_angle", "max_pitch_angle", "max_yaw_rate", "manual_command_decay")
    return fnv1a32(b"".join(np.asarray(model[name], dtype=np.float32).tobytes() for name in names))


def literal(value: float) -> str:
    rendered = f"{float(value):.9g}"
    if "." not in rendered and "e" not in rendered.lower():
        rendered += ".0"
    return rendered + "f"


def cpp_string(value: str) -> str:
    escaped = (
        value.replace("\\", "\\\\")
        .replace('"', '\\"')
        .replace("\n", "\\n")
        .replace("\r", "\\r")
        .replace("\t", "\\t")
    )
    return f'"{escaped}"'


def array_lines(name: str, values: np.ndarray) -> list[str]:
    result = [f"static constexpr float {name}[{values.size}] = {{"]
    result.extend(f"\t{literal(value)}," for value in values.ravel())
    result.append("};")
    return result


def write_model_header(
    model: dict[str, np.ndarray | np.float32],
    controller_config: Path,
) -> int:
    digest = checksum(model)
    motor_table_points = int(np.asarray(model["motor_table"]).size // 2)
    if motor_table_points < 2 or np.asarray(model["motor_table"]).size % 2:
        raise ValueError("nominal motor table must contain command/speed pairs")
    weight_bytes = int(
        sum(
            np.asarray(model[name]).size
            for name in (
                "gain",
                "decay",
                "slope_eff",
                "integral_limit",
                "motor_table",
                "manual_command_decay",
            )
        )
        * 4
        + 9 * 4
    )
    source = (
        controller_config.relative_to(ROOT)
        if controller_config.is_relative_to(ROOT)
        else controller_config
    )
    lines = [
        "/****************************************************************************", " * Generated by px4_trans/tools/generate_lqi_backend.py. Do not edit.",
        f" * Source: {source}", " ****************************************************************************/", "", "#pragma once", "",
        '#include "LqiControllerModel.hpp"', "", "namespace lqi_nominal", "{", "",
    ]
    for name in ("gain", "decay", "slope_eff", "integral_limit", "motor_table", "manual_command_decay"):
        lines += array_lines("k" + "".join(part.title() for part in name.split("_")), np.asarray(model[name])) + [""]
    lines += [
        "static constexpr LqiControllerModel kModel {",
        "\tLqiControllerModel::kSchemaVersion,", "\tLqiControllerModel::WorldFrame::NED,", "\tLqiControllerModel::BodyFrame::FRD,",
        "\tLqiControllerModel::QuaternionConvention::HamiltonWxyzBodyToWorld,", "\tLqiControllerModel::CollectiveContract::ExternalUpperLqiLowerAndServos,",
        "\tLqiControllerModel::ActuatorObserver::MotorRpmOrCommandDriven,", "\tkGain,", "\tkDecay,", "\tkSlopeEff,", "\tkIntegralLimit,", "\tkMotorTable,", f"\t{motor_table_points}u,",
        f"\t{literal(model['dt'])},", f"\t{literal(model['upper_trim'])},", f"\t{literal(model['lower_trim'])},",
        f"\t{literal(model['upper_trim_rad_s'])},", f"\t{literal(model['lower_trim_rad_s'])},", f"\t{literal(model['rpm_to_rad_s'])},",
        f"\t{literal(model['max_roll_angle'])},", f"\t{literal(model['max_pitch_angle'])},", f"\t{literal(model['max_yaw_rate'])},", "\tkManualCommandDecay,",
        f"\t{weight_bytes}u,", f"\t0x{digest:08x}u,", "};", "", "} // namespace lqi_nominal", "",
    ]
    MODEL_HEADER.write_text("\n".join(lines), encoding="utf-8")
    return digest


def write_golden(vectors: list[dict[str, object]]) -> None:
    fields = [
        "\tfloat attitude_q_wb[4];", "\tfloat target_attitude_q_wb[4];", "\tfloat angular_velocity_b[3];",
        "\tfloat target_angular_velocity_b[3];", "\tfloat collective_base;", "\tfloat motor_rpm[2];", "\tbool motor_rpm_valid;",
        "\tfloat persistent[8];", "\tfloat expected_output[4];", "\tfloat expected_upper;", "\tfloat expected_next_persistent[8];",
    ]
    lines = ["#pragma once", "", "struct LqiGoldenVector {"] + fields + ["};", "", "static constexpr LqiGoldenVector kLqiGoldenVectors[] = {"]
    array_names = ("attitude_q_wb", "target_attitude_q_wb", "angular_velocity_b", "target_angular_velocity_b")
    for vector in vectors:
        lines.append("\t{")
        for name in array_names:
            lines.append("\t\t{" + ", ".join(literal(v) for v in vector[name]) + "},")
        lines.append(f"\t\t{literal(vector['collective_base'])},")
        lines.append("\t\t{" + ", ".join(literal(v) for v in vector["motor_rpm"]) + "},")
        lines.append("\t\t" + ("true," if vector["motor_rpm_valid"] else "false,"))
        for name in ("persistent", "expected_output"):
            lines.append("\t\t{" + ", ".join(literal(v) for v in vector[name]) + "},")
        lines.append(f"\t\t{literal(vector['expected_upper'])},")
        lines.append("\t\t{" + ", ".join(literal(v) for v in vector["expected_next_persistent"]) + "},")
        lines.append("\t},")
    lines += ["};", "", f"static constexpr int kLqiGoldenVectorCount{{{len(vectors)}}};", ""]
    GOLDEN_HEADER.write_text("\n".join(lines), encoding="utf-8")
    GOLDEN_JSON.write_text(json.dumps({"schema_version": 2, "vectors": vectors}, indent=2), encoding="utf-8")


def load_identified_model(
    identified_json: Path,
    nominal: dict[str, np.ndarray | np.float32],
    checkpoint_path: Path,
    *,
    airframe_record: dict[str, object],
) -> dict[str, np.ndarray | np.float32]:
    """Build the 19-state composite payload from the offline identification."""

    report = json.loads(identified_json.read_text(encoding="utf-8"))
    airframe = airframe_provenance(airframe_record)
    validate_airframe_payload(
        report,
        str(airframe["airframe_id"]),
        artifact_name=str(identified_json),
    )
    bundle_path = ROOT / str(report["bundle"])
    expected_bundle_sha256 = report.get("bundle_sha256")
    if not isinstance(expected_bundle_sha256, str):
        raise ValueError("identified report must record bundle_sha256")
    if file_sha256(bundle_path) != expected_bundle_sha256:
        raise ValueError("identification report bundle_sha256 does not match data")
    axis_checkpoint_sha256 = report.get("axis_checkpoint_sha256", {})
    for axis, path_value in report.get("axis_checkpoints", {}).items():
        expected = axis_checkpoint_sha256.get(axis)
        if not isinstance(expected, str):
            raise ValueError(
                f"identified report must record axis checkpoint hash for {axis}"
            )
        if file_sha256(ROOT / str(path_value)) != expected:
            raise ValueError(
                f"identified report {axis} checkpoint hash does not match file"
            )
    aggregate = report["aggregate"]
    gain = np.asarray(aggregate["analysis_gain"], dtype=np.float32)
    coefficients = np.asarray(
        aggregate["predicted_composite_coefficients"], dtype=np.float32
    )
    if gain.shape != (4, 19):
        raise ValueError(f"identified gain must be 4x19, got {gain.shape}")
    if coefficients.shape != (3, 3, 3):
        raise ValueError(
            f"identified coefficients must be (3,3,3), got {coefficients.shape}"
        )
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    mode_transform = np.asarray(checkpoint["mode_transform"], dtype=np.float32)
    servo_slopes = np.asarray(checkpoint["servo_command_slopes"], dtype=np.float32)
    basis_tau = np.asarray(
        tuple(float(value) for value in checkpoint["basis_time_constants_s"]),
        dtype=np.float32,
    )
    if mode_transform.shape != (3, 3) or servo_slopes.shape != (3,):
        raise ValueError("checkpoint mode_transform/slopes have unexpected shape")
    latent_decay = np.exp(-DT_S / basis_tau).astype(np.float32)
    servo_input = (mode_transform @ np.diag(servo_slopes)).astype(np.float32)
    latent_input = np.zeros((3, 3, 3), dtype=np.float32)
    for basis in range(3):
        latent_input[basis] = servo_input * (1.0 - latent_decay[basis])
    return {
        "gain": gain,
        "motor_decay": np.asarray(nominal["decay"])[:2].astype(np.float32),
        "motor_slope_eff": np.asarray(nominal["slope_eff"])[:2].astype(np.float32),
        "latent_decay": latent_decay,
        "latent_input": latent_input,
        "integral_limit": np.asarray(nominal["integral_limit"], dtype=np.float32),
        "motor_table": np.asarray(nominal["motor_table"], dtype=np.float32),
        "dt": f32(DT_S),
        "upper_trim": f32(float(nominal["upper_trim"])),
        "lower_trim": f32(float(nominal["lower_trim"])),
        "upper_trim_rad_s": f32(float(nominal["upper_trim_rad_s"])),
        "lower_trim_rad_s": f32(float(nominal["lower_trim_rad_s"])),
        "rpm_to_rad_s": f32(RPM_TO_RAD_S),
        "max_roll_angle": f32(float(nominal["max_roll_angle"])),
        "max_pitch_angle": f32(float(nominal["max_pitch_angle"])),
        "max_yaw_rate": f32(float(nominal["max_yaw_rate"])),
        "manual_command_decay": np.asarray(
            nominal["manual_command_decay"], dtype=np.float32
        ),
    }


def lqi_composite_step_float32(
    current_q: np.ndarray,
    target_q: np.ndarray,
    current_rate: np.ndarray,
    target_rate: np.ndarray,
    collective: float,
    motor_rpm: np.ndarray,
    motor_rpm_valid: bool,
    persistent: np.ndarray,
    model: dict[str, np.ndarray | np.float32],
) -> tuple[np.ndarray, np.ndarray, np.float32]:
    """Python mirror of LqiCompositeCore::stepFromState (persistent[14])."""

    error = quaternion_error(current_q, target_q)
    rate_error = (current_rate - target_rate).astype(np.float32)
    collective_f = np.clip(f32(collective), f32(0.0), f32(1.0)).astype(np.float32)
    if collective_f <= f32(0.0):
        return (
            np.zeros(4, dtype=np.float32),
            persistent.copy(),
            collective_f,
        )
    target_speed = f32(
        np.interp(
            float(collective_f),
            model["motor_table"][:, 0],
            model["motor_table"][:, 1],
        )
    )
    target_dev = target_speed - np.asarray(
        [model["upper_trim_rad_s"], model["lower_trim_rad_s"]], dtype=np.float32
    )
    if motor_rpm_valid:
        motor_error = (
            np.abs(motor_rpm).astype(np.float32) * model["rpm_to_rad_s"]
            - np.asarray(
                [model["upper_trim_rad_s"], model["lower_trim_rad_s"]],
                dtype=np.float32,
            )
            - target_dev
        )
    else:
        motor_error = persistent[:2] - target_dev
    state = np.concatenate(
        (error[:2], rate_error, motor_error, persistent[2:11], persistent[11:14])
    ).astype(np.float32)
    delta = np.empty(4, dtype=np.float32)
    for row in range(4):
        accumulator = f32(0.0)
        for col in range(19):
            accumulator = f32(accumulator + f32(model["gain"][row, col]) * state[col])
        delta[row] = -accumulator
    raw = np.concatenate((collective_f + delta[:1], delta[1:])).astype(np.float32)
    lower_min = np.maximum(collective_f - f32(0.2), f32(0.0)).astype(np.float32)
    lower_max = np.minimum(collective_f + f32(0.2), f32(1.0)).astype(np.float32)
    output = np.concatenate(
        (np.clip(raw[:1], lower_min, lower_max), np.clip(raw[1:], f32(-1.0), f32(1.0)))
    ).astype(np.float32)
    saturated = bool(
        np.any(raw[:1] < lower_min)
        or np.any(raw[:1] > lower_max)
        or np.any(raw[1:] < -1.0)
        or np.any(raw[1:] > 1.0)
    )
    integral_error = np.asarray([error[0], error[1], rate_error[2]], dtype=np.float32)
    candidate = np.clip(
        persistent[11:14] + integral_error * model["dt"],
        -model["integral_limit"],
        model["integral_limit"],
    ).astype(np.float32)
    motor_command_dev = np.asarray(
        [
            collective_f - model["upper_trim"],
            output[0] - model["lower_trim"],
        ],
        dtype=np.float32,
    )
    motor_next = (
        model["motor_decay"] * persistent[:2]
        + model["motor_slope_eff"] * motor_command_dev
    ).astype(np.float32)
    if motor_rpm_valid:
        motor_next[:2] = (
            np.abs(motor_rpm).astype(np.float32) * model["rpm_to_rad_s"]
            - np.asarray(
                [model["upper_trim_rad_s"], model["lower_trim_rad_s"]],
                dtype=np.float32,
            )
        )
    latent_next = np.zeros(9, dtype=np.float32)
    for basis in range(3):
        for axis in range(3):
            value = model["latent_decay"][basis] * persistent[2 + 3 * basis + axis]
            for servo in range(3):
                value += (
                    model["latent_input"][basis, axis, servo] * output[1 + servo]
                )
            latent_next[3 * basis + axis] = f32(value)
    integral_next = persistent[11:14] if saturated else candidate
    next_persistent = np.concatenate(
        (motor_next, latent_next, integral_next)
    ).astype(np.float32)
    return output, next_persistent, collective_f


def composite_golden_vectors(
    model: dict[str, np.ndarray | np.float32],
) -> list[dict[str, object]]:
    cases = [
        ([1, 0, 0, 0], [1, 0, 0, 0], [0, 0, 0], [0, 0, 0], 0.5, [0, 0], False, [0] * 14),
        ([0.9998, 0.01, 0, 0], [1, 0, 0, 0], [0.1, -0.2, 0.05], [0, 0, 0.3], 0.75,
         [11000, -11500], True, [0] * 14),
        ([0.9, 0.2, -0.3, 0.1], [0.98, -0.1, 0.12, 0.02], [-1, 0.8, 0.6], [0, 0, -0.5], 0.2,
         [9800, 10200], True, [100, -50, 0.05, -0.02, 0.03, 0.1, -0.2, 0.3, 0.01, -0.05, 0.07, 0.1, -0.2, 0.3]),
        ([-0.9998, -0.01, 0, 0], [1, 0, 0, 0], [0.3, 0.4, 0.2], [0, 0, 1.2], 1.0,
         [0, 0], False, [500, 400, 0.3, 0.25, 0.2, 0.2, 0.2, 0.4, -0.1, 0.15, -0.2, 0.2, 0.2, 0.4]),
        ([0.8, 0.4, 0.4, 0.2], [0.9239, 0, 0, 0.3827], [3, -3, 2], [0, 0, 0], 0.0,
         [13500, 9000], True, [-300, 200, -0.3, 0, 0.1, -0.25, 0.15, -0.45, 0.2, -0.1, 0.05, -0.25, 0.15, -0.45]),
    ]
    vectors = []
    for current_q, target_q, current_rate, target_rate, collective, rpm, valid, persistent in cases:
        arrays = [
            np.asarray(value, dtype=np.float32)
            for value in (current_q, target_q, current_rate, target_rate, rpm, persistent)
        ]
        output, next_state, upper = lqi_composite_step_float32(
            *arrays[:4], collective, arrays[4], valid, arrays[5], model
        )
        vectors.append(
            {
                "attitude_q_wb": arrays[0].tolist(),
                "target_attitude_q_wb": arrays[1].tolist(),
                "angular_velocity_b": arrays[2].tolist(),
                "target_angular_velocity_b": arrays[3].tolist(),
                "collective_base": float(collective),
                "motor_rpm": arrays[4].tolist(),
                "motor_rpm_valid": valid,
                "persistent": arrays[5].tolist(),
                "expected_output": output.tolist(),
                "expected_upper": float(upper),
                "expected_next_persistent": next_state.tolist(),
            }
        )
    return vectors


def composite_checksum(model: dict[str, np.ndarray | np.float32]) -> int:
    names = (
        "gain",
        "motor_decay",
        "motor_slope_eff",
        "latent_decay",
        "latent_input",
        "integral_limit",
        "motor_table",
        "dt",
        "upper_trim",
        "lower_trim",
        "upper_trim_rad_s",
        "lower_trim_rad_s",
        "rpm_to_rad_s",
        "max_roll_angle",
        "max_pitch_angle",
        "max_yaw_rate",
        "manual_command_decay",
    )
    return fnv1a32(
        b"".join(np.asarray(model[name], dtype=np.float32).tobytes() for name in names)
    )


def write_composite_model_header(
    model: dict[str, np.ndarray | np.float32],
    identified_json: Path,
    *,
    airframe: dict[str, object],
    identification_artifact_sha256: str,
) -> int:
    digest = composite_checksum(model)
    motor_table_points = int(np.asarray(model["motor_table"]).size // 2)
    if motor_table_points < 2 or np.asarray(model["motor_table"]).size % 2:
        raise ValueError("identified motor table must contain command/speed pairs")
    source = (
        identified_json.relative_to(ROOT)
        if identified_json.is_relative_to(ROOT)
        else identified_json
    )
    weight_bytes = int(
        sum(np.asarray(model[name]).size for name in (
            "gain",
            "motor_decay",
            "motor_slope_eff",
            "latent_decay",
            "latent_input",
            "integral_limit",
            "motor_table",
            "manual_command_decay",
        ))
        * 4
        + 9 * 4
    )
    lines = [
        "/****************************************************************************",
        " * Generated by px4_trans/tools/generate_lqi_backend.py. Do not edit.",
        f" * Source: {source}",
        f" * Airframe: {airframe['airframe_id']} ({airframe['display_name']})",
        f" * Identification artifact SHA-256: {identification_artifact_sha256}",
        " ****************************************************************************/",
        "",
        "#pragma once",
        "",
        '#include "LqiCompositeControllerModel.hpp"',
        "",
        "namespace lqi_identified",
        "{",
        "",
        f"static constexpr uint32_t kAirframeNumber{{{int(airframe['airframe_number'])}u}};",
        f"static constexpr char kAirframeId[]{{{cpp_string(str(airframe['airframe_id']))}}};",
        f"static constexpr char kPlatformFamily[]{{{cpp_string(str(airframe['platform_family']))}}};",
        f"static constexpr char kIdentificationArtifactSha256[]{{{cpp_string(identification_artifact_sha256)}}};",
        "",
    ]
    for name in (
        "gain",
        "motor_decay",
        "motor_slope_eff",
        "latent_decay",
        "latent_input",
        "integral_limit",
        "motor_table",
        "manual_command_decay",
    ):
        lines += array_lines(
            "k" + "".join(part.title() for part in name.split("_")),
            np.asarray(model[name]),
        )
        lines += [""]
    lines += [
        "static constexpr LqiCompositeControllerModel kModel {",
        "\tLqiCompositeControllerModel::kSchemaVersion,",
        "\tLqiCompositeControllerModel::WorldFrame::NED,",
        "\tLqiCompositeControllerModel::BodyFrame::FRD,",
        "\tLqiCompositeControllerModel::QuaternionConvention::HamiltonWxyzBodyToWorld,",
        "\tLqiCompositeControllerModel::CollectiveContract::ExternalUpperLqiLowerAndServos,",
        "\tLqiCompositeControllerModel::ActuatorObserver::MotorRpmOrCommandDriven,",
        "\tkGain,", "\tkMotorDecay,", "\tkMotorSlopeEff,", "\tkLatentDecay,", "\tkLatentInput,",
        "\tkIntegralLimit,", "\tkMotorTable,", f"\t{motor_table_points}u,",
        f"\t{literal(model['dt'])},", f"\t{literal(model['upper_trim'])},", f"\t{literal(model['lower_trim'])},",
        f"\t{literal(model['upper_trim_rad_s'])},", f"\t{literal(model['lower_trim_rad_s'])},", f"\t{literal(model['rpm_to_rad_s'])},",
        f"\t{literal(model['max_roll_angle'])},", f"\t{literal(model['max_pitch_angle'])},", f"\t{literal(model['max_yaw_rate'])},", "\tkManualCommandDecay,",
        f"\t{weight_bytes}u,", f"\t0x{digest:08x}u,", "};", "",
        "} // namespace lqi_identified", "",
    ]
    COMPOSITE_MODEL_HEADER.write_text("\n".join(lines), encoding="utf-8")
    return digest


def write_composite_golden(
    vectors: list[dict[str, object]],
    *,
    airframe: dict[str, object],
) -> None:
    fields = [
        "\tfloat attitude_q_wb[4];", "\tfloat target_attitude_q_wb[4];", "\tfloat angular_velocity_b[3];",
        "\tfloat target_angular_velocity_b[3];", "\tfloat collective_base;", "\tfloat motor_rpm[2];", "\tbool motor_rpm_valid;",
        "\tfloat persistent[14];", "\tfloat expected_output[4];", "\tfloat expected_upper;", "\tfloat expected_next_persistent[14];",
    ]
    lines = [
        "/* Generated identified-LQI vectors.",
        f" * Airframe: {airframe['airframe_id']} ({airframe['display_name']})",
        " */",
        "",
        "#pragma once", "",
        "struct LqiIdentifiedGoldenVector {",
    ] + fields + ["};", "", "static constexpr LqiIdentifiedGoldenVector kLqiIdentifiedGoldenVectors[] = {"]
    array_names = ("attitude_q_wb", "target_attitude_q_wb", "angular_velocity_b", "target_angular_velocity_b")
    for vector in vectors:
        lines.append("\t{")
        for name in array_names:
            lines.append("\t\t{" + ", ".join(literal(v) for v in vector[name]) + "},")
        lines.append(f"\t\t{literal(vector['collective_base'])},")
        lines.append("\t\t{" + ", ".join(literal(v) for v in vector["motor_rpm"]) + "},")
        lines.append("\t\t" + ("true," if vector["motor_rpm_valid"] else "false,"))
        for name in ("persistent", "expected_output"):
            lines.append("\t\t{" + ", ".join(literal(v) for v in vector[name]) + "},")
        lines.append(f"\t\t{literal(vector['expected_upper'])},")
        lines.append("\t\t{" + ", ".join(literal(v) for v in vector["expected_next_persistent"]) + "},")
        lines.append("\t},")
    lines += ["};", "", f"static constexpr int kLqiIdentifiedGoldenVectorCount{{{len(vectors)}}};", ""]
    COMPOSITE_GOLDEN_HEADER.write_text("\n".join(lines), encoding="utf-8")
    COMPOSITE_GOLDEN_JSON.write_text(
        json.dumps(
            {"schema_version": 3, "airframe": airframe, "vectors": vectors},
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )


def relative_path(path: Path) -> str:
    resolved = path.expanduser().resolve()
    return (
        resolved.relative_to(ROOT).as_posix()
        if resolved.is_relative_to(ROOT)
        else str(resolved)
    )


def write_composite_manifest(
    *,
    model: dict[str, np.ndarray | np.float32],
    airframe_record: dict[str, object],
    deployment_config: Path,
    identified_json: Path,
    checkpoint_path: Path,
    header_path: Path,
    archived_header: Path,
    model_checksum: int,
) -> Path:
    airframe = airframe_provenance(airframe_record)
    manifest_path = (
        EMBEDDED_MANIFEST_ROOT
        / str(airframe["airframe_id"])
        / "lqi_embedded_model.json"
    )
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "artifact_type": "px4_embedded_lqi_model",
        "airframe": airframe,
        "model_index": 1,
        "model_checksum": f"0x{model_checksum:08x}",
        "required_parameter": {
            "name": "NN_AIRFRAME_ID",
            "value": int(airframe["airframe_number"]),
        },
        "sources": {
            "deployment_config": relative_path(deployment_config),
            "identification_report": relative_path(identified_json),
            "identification_report_sha256": file_sha256(identified_json),
            "binding_checkpoint": relative_path(checkpoint_path),
            "binding_checkpoint_sha256": file_sha256(checkpoint_path),
        },
        "generated": {
            "active_header": relative_path(header_path),
            "active_header_sha256": file_sha256(header_path),
            "archived_header": relative_path(archived_header),
            "archived_header_sha256": file_sha256(archived_header),
            "golden_header": relative_path(COMPOSITE_GOLDEN_HEADER),
            "golden_header_sha256": file_sha256(COMPOSITE_GOLDEN_HEADER),
            "golden_json": relative_path(COMPOSITE_GOLDEN_JSON),
            "golden_json_sha256": file_sha256(COMPOSITE_GOLDEN_JSON),
        },
        "weight_bytes": int(
            sum(
                np.asarray(model[name]).size
                for name in (
                    "gain",
                    "motor_decay",
                    "motor_slope_eff",
                    "latent_decay",
                    "latent_input",
                    "integral_limit",
                    "motor_table",
                    "manual_command_decay",
                )
            )
            * 4
            + 9 * 4
        ),
    }
    manifest_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return manifest_path


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--deployment-config",
        type=Path,
        default=DEPLOYMENT_CONFIG,
        help="deployment manifest (default: px4_trans/configs/lqi_airframe_1.yaml)",
    )
    parser.add_argument(
        "--identified-json",
        type=Path,
        help="override the deployment manifest identification report",
    )
    parser.add_argument(
        "--identified-checkpoint",
        type=Path,
        help="override the checkpoint carrying mode/slope/basis metadata",
    )
    parser.add_argument("--airframe-id")
    parser.add_argument("--airframe-registry", type=Path)
    parser.add_argument("--output-header", type=Path, default=COMPOSITE_MODEL_HEADER)
    parser.add_argument(
        "--no-identified",
        action="store_true",
        help="regenerate only the airframe-independent nominal 13-state model",
    )
    args = parser.parse_args()

    deployment_config = args.deployment_config.expanduser().resolve()
    deployment = yaml.safe_load(deployment_config.read_text(encoding="utf-8"))
    if not isinstance(deployment, dict):
        raise ValueError("deployment manifest must contain a mapping")
    model = load_model(deployment_config)
    controller_config = (
        ROOT / deployment["controller_config"]
    ).resolve()
    nominal_digest = checksum(model)
    nominal_vectors = golden_vectors(model)

    if args.no_identified:
        write_model_header(model, controller_config)
        write_golden(nominal_vectors)
        print(
            f"wrote nominal 4x13 model (checksum 0x{nominal_digest:08x}) "
            f"and {len(nominal_vectors)} golden vectors"
        )
        return

    identified_config = deployment.get("identified_model")
    if not isinstance(identified_config, Mapping):
        raise ValueError(
            "deployment manifest must define identified_model or be generated "
            "with --no-identified"
        )
    identified_path = Path(
        args.identified_json
        if args.identified_json is not None
        else identified_config["report"]
    ).expanduser()
    identified_path = (
        identified_path
        if identified_path.is_absolute()
        else (ROOT / identified_path)
    ).resolve()
    checkpoint_path = Path(
        args.identified_checkpoint
        if args.identified_checkpoint is not None
        else identified_config["binding_checkpoint"]
    ).expanduser()
    checkpoint_path = (
        checkpoint_path
        if checkpoint_path.is_absolute()
        else (ROOT / checkpoint_path)
    ).resolve()
    registry_path = (
        args.airframe_registry
        if args.airframe_registry is not None
        else (
            ROOT / deployment["airframe_registry"]
            if "airframe_registry" in deployment
            else DEFAULT_REGISTRY
        )
    ).expanduser().resolve()
    report = json.loads(identified_path.read_text(encoding="utf-8"))
    report_airframe_value = report.get("airframe")
    if not isinstance(report_airframe_value, Mapping):
        raise ValueError("identified report has no airframe provenance")
    report_airframe = validate_airframe_payload(
        report,
        str(report_airframe_value["airframe_id"]),
        artifact_name=str(identified_path),
    )
    airframe_id = (
        args.airframe_id
        or deployment.get("airframe_id")
        or report_airframe["airframe_id"]
    )
    airframe_record = resolve_airframe(str(airframe_id), registry_path)
    airframe = airframe_provenance(airframe_record)
    if airframe["airframe_id"] != report_airframe["airframe_id"]:
        raise ValueError(
            f"deployment airframe {airframe['airframe_id']} does not match "
            f"identification report airframe {report_airframe['airframe_id']}"
        )
    if (
        deployment.get("airframe_id") is not None
        and deployment["airframe_id"] != airframe["airframe_id"]
    ):
        raise ValueError("deployment manifest airframe_id is inconsistent")
    identified = load_identified_model(
        identified_path,
        model,
        checkpoint_path,
        airframe_record=airframe_record,
    )
    expected_checksum = identified_config.get("expected_checksum")
    composite_digest = composite_checksum(identified)
    if expected_checksum is not None:
        expected = (
            int(expected_checksum, 16)
            if isinstance(expected_checksum, str)
            else int(expected_checksum)
        )
        if composite_digest != expected:
            raise ValueError(
                f"generated identified checksum 0x{composite_digest:08x} does not "
                f"match deployment expectation 0x{expected:08x}"
            )
    identification_sha256 = file_sha256(identified_path)
    header_path = args.output_header.expanduser()
    header_path = (
        header_path if header_path.is_absolute() else ROOT / header_path
    ).resolve()
    if header_path != COMPOSITE_MODEL_HEADER:
        raise ValueError(
            "non-default identified header output is not wired into the firmware build; "
            "use a separate worktree/deployment config instead"
        )
    write_model_header(model, controller_config)
    write_golden(nominal_vectors)
    composite_digest = write_composite_model_header(
        identified,
        identified_path,
        airframe=airframe,
        identification_artifact_sha256=identification_sha256,
    )
    composite_vectors = composite_golden_vectors(identified)
    write_composite_golden(composite_vectors, airframe=airframe)
    archived_header = (
        EMBEDDED_MANIFEST_ROOT
        / str(airframe["airframe_id"])
        / "LqiIdentifiedModel.hpp"
    )
    archived_header.parent.mkdir(parents=True, exist_ok=True)
    archived_header.write_text(
        COMPOSITE_MODEL_HEADER.read_text(encoding="utf-8"), encoding="utf-8"
    )
    manifest_path = write_composite_manifest(
        model=identified,
        airframe_record=airframe_record,
        deployment_config=deployment_config,
        identified_json=identified_path,
        checkpoint_path=checkpoint_path,
        header_path=COMPOSITE_MODEL_HEADER,
        archived_header=archived_header,
        model_checksum=composite_digest,
    )
    print(
        f"wrote nominal 4x13 model (checksum 0x{nominal_digest:08x}) "
        f"and {len(nominal_vectors)} golden vectors"
    )
    print(
        f"wrote identified 4x19 composite model for {airframe['airframe_id']} "
        f"(checksum 0x{composite_digest:08x}) and {len(composite_vectors)} golden vectors"
    )
    print(f"wrote embedded model manifest: {manifest_path}")


if __name__ == "__main__":
    main()
