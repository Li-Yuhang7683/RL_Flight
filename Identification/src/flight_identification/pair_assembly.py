"""双机体刚性拼接协同控制的第一级可行性烟测。

目的:

- 用标称 sim2real 机体参数，把两台同样的机体刚性拼接为一个刚体，形成
  "双执行器单刚体"模型: 质量叠加、平行轴惯量、每台机的格栅力/力矩以
  ``r x F`` 力臂方式合成到拼接体系。
- 用与单机部署相同的路径合成协同增益（连续雅可比 -> ZOH 离散化 -> DARE）:
  单机 = K4 [4,13]（上桨外部，飞手所有），双机 = K8 [8,18]。
- 权限上界在真实（分段线性/非线性）执行器映射上做多起点优化给出（每轴最大
  力矩/角加速度），闭环行为用 500 Hz 旋转通道烟测，判据沿用现有 2 度 /
  0.1 rad/s / 0.5 s hold 与 60 度安全倾角。

边界（明确不做）:

- 只建模旋转通道；上桨油门取悬停值恒定（无定高环），无传感器噪声/延迟，
  无平移/高度耦合；
- 气动按标称参数（含格栅自身衰减与耦合衰减）；不含两机间气动干扰、
  结构柔性、装配偏差；
- 结论等级为 design_only 烟测，不构成任何飞行/部署结论。

布局命名说明:

- ``side`` / ``tandem`` 只是"拼接连线相对单元机体 x/y 轴"的标签。若机体真正
  轴对称（C-infinity），两者是同一构型的坐标重标，力学等价，只有拼接距离 d
  有意义。
- 标称机体是三格栅 120 度布局（C3，带镜像），90 度旋转不是严格对称；实测
  水平力矩包络在强轴与对角方向几乎一致，只有弱轴相差约 15%。真实装配误差
  （质心/推力中心/格栅方位）与前飞两机气动干扰才是"拼接方位"成为设计变量的
  来源。

复用:

- 执行器非线性与逐单元力/力矩复用 ``SimEnv`` 的 ``TensorDynamicsKernel``
  （电机死区/一阶惯性、舵机死区/回差/限速、格栅矢量推力与耦合衰减）；
- 逐单元稳态力/力矩函数逐项复刻 ``flight_controller.plant``，运行时断言与
  ``LocalPlantModel.wrench_from_actuators`` 一致。
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np
import torch
from scipy.linalg import expm

from flight_controller import (
    ControllerContext,
    create_controller,
    load_controller_config,
)
from flight_controller.math import (
    lookup,
    quaternion_multiply,
    quaternion_rotation_error,
    tilt_cosine,
)
from flight_controller.plant import LocalPlantModel
from simenv.config import load_and_materialize
from simenv.dynamics import TensorDynamicsKernel

from .control_evaluation import _lqr_gain


DT = 1.0 / 500.0
CONVERGENCE_ROLL_PITCH_RAD = 0.0349066  # 2 deg
CONVERGENCE_RATE_RAD_S = 0.10
CONVERGENCE_HOLD_S = 0.5
SAFETY_TILT_RAD = 1.0471976  # 60 deg
SAFETY_RATE_RAD_S = 10.0

DEFAULT_UNIT_CONFIG = Path("SimEnv/configs/sim2real_micro_coaxial.yaml")
DEFAULT_CONTROLLER_CONFIG = Path("Controller/configs/lqi_sim2real_micro_coaxial_4out.yaml")
DEFAULT_OUTPUT = Path("Identification/runs/pair_assembly_feasibility_v1/audit.json")

# (roll_deg, pitch_deg, p, q, r)。
# 对 (roll,pitch,p,q) <-> (pitch,roll,q,p) 成对对称，避免初值集偏向某一轴，
# 否则会把"侧拼 vs 串列"的差异误读成布局优劣。
DEFAULT_INITIAL_CONDITIONS: tuple[tuple[float, float, float, float, float], ...] = (
    (15.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 15.0, 0.0, 0.0, 0.0),
    (10.0, 10.0, 0.5, 0.5, 0.0),
    (15.0, -10.0, -0.5, 0.5, 0.5),
    (-10.0, 15.0, 0.5, -0.5, 0.5),
    (5.0, -5.0, 1.0, -1.0, 1.0),
    (-5.0, 5.0, -1.0, 1.0, 1.0),
)

# 配对感知的权重重调候选: 名字 -> 覆盖项。
RETUNE_VARIANTS: tuple[tuple[str, dict[str, float]], ...] = (
    ("single_weights", {}),
    ("yaw_rate_0p5", {"yaw_rate_scale": 0.5}),
    ("yaw_rate_0p5_input_0p04", {"yaw_rate_scale": 0.5, "input_weight_scale": 0.04}),
    (
        "relaxed_rp_0p12_yaw_0p35",
        {"yaw_rate_scale": 0.35, "roll_pitch_scale": 0.12, "input_weight_scale": 0.04},
    ),
)


@dataclass(frozen=True)
class AssemblyLayout:
    """拼接布局: 每个单元质心在拼接体系（机体系 FRD）中的位置。"""

    name: str
    offsets: np.ndarray


def single_layout() -> AssemblyLayout:
    return AssemblyLayout("single", np.zeros((1, 3), dtype=np.float64))


def pair_layout(direction: str, half_separation_m: float) -> AssemblyLayout:
    half = float(half_separation_m)
    if half <= 0.0:
        raise ValueError("half separation must be positive")
    if direction == "side":
        offsets = np.array([[0.0, -half, 0.0], [0.0, half, 0.0]])
    elif direction == "tandem":
        offsets = np.array([[-half, 0.0, 0.0], [half, 0.0, 0.0]])
    else:
        raise ValueError("direction must be side or tandem")
    return AssemblyLayout(f"pair_{direction}_{half:.3f}", offsets)


def unit_force_moment(
    parameters: Mapping[str, torch.Tensor],
    motor_speed: torch.Tensor,
    servo_angle: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    """复刻 ``LocalPlantModel.wrench_from_actuators``，并额外返回合力。

    与 plant.py 逐项一致: 直接推力、格栅矢量推力（含自身衰减与耦合衰减）、
    上下桨反扭矩差；力矩作用于各单元自身质心。
    """

    coefficients = parameters["aerodynamics.thrust_coefficients"]
    total_thrust = (
        coefficients[:, 0] * motor_speed[:, 0].square()
        + coefficients[:, 1] * motor_speed[:, 1].square()
        + coefficients[:, 2] * motor_speed[:, 0] * motor_speed[:, 1]
    )
    neutral = parameters["aerodynamics.neutral_thrust_direction_b"]
    partition = parameters["aerodynamics.thrust_partition"]
    direct_force = total_thrust[:, None] * partition[:, :1] * neutral
    direct_arm = (
        parameters["aerodynamics.direct_thrust_center_b"]
        - parameters["body.center_of_mass_b"]
    )
    direct_moment = torch.linalg.cross(direct_arm, direct_force)

    attenuation = lookup(
        servo_angle.abs(),
        parameters["aerodynamics.grids.self_attenuation_curve"],
    )
    coupling_loss = torch.bmm(
        parameters["aerodynamics.coupling_attenuation"],
        (1.0 - attenuation).unsqueeze(-1),
    ).squeeze(-1)
    effective = (attenuation - coupling_loss).clamp(0.0, 1.0)
    vector_angle = (
        parameters["aerodynamics.grids.vector_deflection.gain"] * servo_angle
        + parameters["aerodynamics.grids.vector_deflection.offset"]
    )
    axes = parameters["aerodynamics.grids.deflection_axis_b"]
    base = neutral[:, None, :].expand(-1, 3, -1)
    cosine = torch.cos(vector_angle)[..., None]
    sine = torch.sin(vector_angle)[..., None]
    projection = (axes * base).sum(dim=-1, keepdim=True)
    direction = (
        base * cosine
        + torch.linalg.cross(axes, base) * sine
        + axes * projection * (1.0 - cosine)
    )
    grid_thrust = total_thrust[:, None] * partition[:, 1:] * effective
    grid_force = grid_thrust[..., None] * direction
    grid_arm = (
        parameters["aerodynamics.grids.aerodynamic_center_b"]
        - parameters["body.center_of_mass_b"][:, None, :]
    )
    grid_moment = torch.linalg.cross(grid_arm, grid_force).sum(dim=1)
    motor_torque = parameters["motors.torque_coefficient"] * motor_speed.square()
    reaction = (motor_torque[:, 0] - motor_torque[:, 1])[:, None] * (-neutral)
    force = direct_force + grid_force.sum(dim=1)
    moment = direct_moment + grid_moment + reaction
    return force, moment


def pair_inertia(
    unit_inertia: torch.Tensor,
    unit_mass: torch.Tensor,
    offsets: np.ndarray,
    device: torch.device,
    dtype: torch.dtype,
) -> torch.Tensor:
    """平行轴定理下的拼接体惯量对角（各单元同向、轴平行）。"""

    inertia = torch.zeros(3, device=device, dtype=dtype)
    mass = unit_mass[0]
    for offset in offsets:
        r = torch.tensor(offset, device=device, dtype=dtype)
        inertia = inertia + unit_inertia[0] + mass * (
            r.dot(r) - r.square()
        )
    return inertia


def _commands_per_unit(trim_command: torch.Tensor) -> torch.Tensor:
    """单机受控输入 (lower pwm, servo_1..3) 在悬停配平处的取值。"""

    return trim_command[0, 1:].clone()


def build_linear_model(
    parameters: Mapping[str, torch.Tensor],
    layout: AssemblyLayout,
    state_scales: Sequence[float],
    integral_scales: Sequence[float],
    input_scales: Sequence[float],
    input_weight_scale: float,
    upper_external: bool = True,
) -> dict[str, Any]:
    """拼装线性模型并合成 LQI 增益（与 ``_design_lqr`` 同构）。"""

    device = parameters["body.mass"].device
    dtype = parameters["body.mass"].dtype
    unit_count = layout.offsets.shape[0]
    plant = LocalPlantModel(parameters)
    trim = plant.hover_trim()
    if upper_external:
        input_per_unit = 4
        u0_unit = _commands_per_unit(trim.command)
    else:
        input_per_unit = 5
        u0_unit = trim.command[0].clone()

    offsets = [
        torch.tensor(offset, device=device, dtype=dtype)
        for offset in layout.offsets
    ]
    inertia = pair_inertia(
        parameters["body.inertia_diagonal_b"],
        parameters["body.mass"],
        layout.offsets,
        device,
        dtype,
    )
    motor_tau = parameters["motors.time_constant"][0]
    servo_tau = parameters["servos.tau"][0]
    motor_table = parameters["motors.pwm_to_rpm_table"]
    servo_table = parameters["servos.pwm_angle_table"]
    upper_pwm = trim.command[0, 0:1]

    state_dim = 5 + 5 * unit_count
    input_dim = input_per_unit * unit_count
    x0 = torch.cat(
        [
            torch.zeros(5, device=device, dtype=dtype),
            *[
                torch.cat(
                    (trim.motor_speed[0], torch.zeros(3, device=device, dtype=dtype))
                )
                for _ in range(unit_count)
            ],
        ]
    )
    u0 = torch.cat([u0_unit for _ in range(unit_count)])

    def continuous(x: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        rates = x[2:5]
        force_total = torch.zeros(3, device=device, dtype=dtype)
        moment_total = torch.zeros(3, device=device, dtype=dtype)
        motor_dots = []
        servo_dots = []
        for index in range(unit_count):
            base = 5 + 5 * index
            motor_speed = x[base : base + 2]
            servo_angle = x[base + 2 : base + 5]
            force, moment = unit_force_moment(
                parameters, motor_speed[None], servo_angle[None]
            )
            force = force[0]
            moment = moment[0]
            force_total = force_total + force
            moment_total = moment_total + moment + torch.linalg.cross(
                offsets[index], force
            )
            unit_u = u[input_per_unit * index : input_per_unit * (index + 1)]
            if upper_external:
                motor_target = lookup(
                    torch.cat((upper_pwm, unit_u[:1]))[None], motor_table
                )[0]
                servo_target = lookup(unit_u[1:][None], servo_table)[0]
            else:
                motor_target = lookup(unit_u[:2][None], motor_table)[0]
                servo_target = lookup(unit_u[2:][None], servo_table)[0]
            motor_dots.append(
                (motor_target - motor_speed) / motor_tau
            )
            servo_dots.append((servo_target - servo_angle) / servo_tau)
        gyroscopic = torch.linalg.cross(rates, inertia * rates)
        angular_acceleration = (moment_total - gyroscopic) / inertia
        return torch.cat(
            (rates[:2], angular_acceleration, *motor_dots, *servo_dots)
        )

    with torch.enable_grad():
        a = torch.autograd.functional.jacobian(
            lambda value: continuous(value, u0), x0
        ).detach().cpu().numpy()
        b = torch.autograd.functional.jacobian(
            lambda value: continuous(x0, value), u0
        ).detach().cpu().numpy()

    integral_output = np.zeros((3, state_dim), dtype=np.float64)
    integral_output[0, 0] = 1.0
    integral_output[1, 1] = 1.0
    integral_output[2, 4] = 1.0
    a = np.block(
        [
            [a, np.zeros((state_dim, 3), dtype=np.float64)],
            [integral_output, np.zeros((3, 3), dtype=np.float64)],
        ]
    )
    b = np.vstack((b, np.zeros((3, input_dim), dtype=np.float64)))
    augmented = np.block(
        [
            [a, b],
            [np.zeros((input_dim, a.shape[0] + input_dim), dtype=np.float64)],
        ]
    )
    discrete = expm(augmented * DT)
    full_state = a.shape[0]
    ad = discrete[:full_state, :full_state]
    bd = discrete[:full_state, full_state:]

    scales = np.asarray(state_scales, dtype=np.float64)
    per_unit = np.concatenate(
        (
            scales[:5],
            np.tile(scales[5:], unit_count),
        )
    )
    q = np.diag(
        1.0 / np.concatenate((per_unit, np.asarray(integral_scales, dtype=np.float64))) ** 2
    )
    r = float(input_weight_scale) * np.diag(
        1.0 / np.tile(np.asarray(input_scales, dtype=np.float64), unit_count) ** 2
    )
    gain = _lqr_gain(ad, bd, q, r)
    poles = np.linalg.eigvals(ad - bd @ gain)
    return {
        "model_sizes": {
            "unit_count": unit_count,
            "state_dim": int(full_state),
            "input_dim": int(input_dim),
        },
        "trim": {
            "motor_speed": trim.motor_speed[0].cpu().numpy().tolist(),
            "command": trim.command[0].cpu().numpy().tolist(),
            "thrust_per_unit_n": float(trim.thrust[0].cpu()),
        },
        "pair_inertia": inertia.cpu().numpy().tolist(),
        "a": ad,
        "b": bd,
        "gain": gain,
        "pole_radius_max": float(np.max(np.abs(poles))),
    }


def synthesize_single_gain(
    parameters: Mapping[str, torch.Tensor],
    state_scales: Sequence[float],
    integral_scales: Sequence[float],
    input_scales: Sequence[float],
    input_weight_scale: float,
) -> dict[str, Any]:
    return build_linear_model(
        parameters,
        single_layout(),
        state_scales,
        integral_scales,
        input_scales,
        input_weight_scale,
    )


def authority_limits(
    parameters: Mapping[str, torch.Tensor],
    layout: AssemblyLayout,
    trim_command: torch.Tensor,
    trim_motor_speed: torch.Tensor,
    inertia: torch.Tensor,
) -> dict[str, Any]:
    """在真实执行器→力矩映射上做多起点优化，给出每轴最大力矩与角加速度。

    直接用分段线性转速表、舵机表与格栅矢量推力映射求最大值，不对配平点切线
    做命令盒外推（下桨转速表在工作点两侧斜率不同，外推会高估权限）。
    """

    device = inertia.device
    dtype = inertia.dtype
    unit_count = layout.offsets.shape[0]
    offsets = [
        torch.tensor(offset, device=device, dtype=dtype) for offset in layout.offsets
    ]
    upper_pwm = trim_command[0]

    def moment_batch(values: np.ndarray) -> np.ndarray:
        """批量真实映射: [n, 4*N] 命令 -> [n, 3] 拼接体系力矩。"""

        rows = []
        for start in range(0, len(values), 512):
            chunk = values[start : start + 512]
            count = len(chunk)
            total_count = count * unit_count
            expanded = {
                name: value.expand(total_count, *value.shape[1:]).contiguous()
                for name, value in parameters.items()
            }
            with torch.no_grad():
                u = torch.tensor(chunk, device=device, dtype=dtype).reshape(
                    count, unit_count, 4
                )
                motor_pwm = torch.stack(
                    (
                        upper_pwm.reshape(1, 1).expand(count, unit_count),
                        u[..., 0].clamp(0.0, 1.0),
                    ),
                    dim=-1,
                ).reshape(total_count, 2)
                speed = lookup(motor_pwm, expanded["motors.pwm_to_rpm_table"])
                speed = torch.where(
                    motor_pwm
                    <= expanded["motors.pwm_deadzone"].reshape(total_count, 2),
                    torch.zeros_like(speed),
                    speed,
                )
                angles = lookup(
                    u[..., 1:].reshape(total_count, 3),
                    expanded["servos.pwm_angle_table"],
                )
                force, moment = unit_force_moment(expanded, speed, angles)
                force = force.reshape(count, unit_count, 3)
                moment = moment.reshape(count, unit_count, 3)
                offsets_t = torch.stack(offsets)[None, :, :]
                total = (
                    moment + torch.linalg.cross(offsets_t, force)
                ).sum(dim=1)
                rows.append(total.cpu().numpy())
        return np.concatenate(rows, axis=0)

    base_lower = float(trim_command[1])
    bounds: list[tuple[float, float]] = []
    for _ in range(unit_count):
        bounds.extend(
            [
                (-base_lower, 1.0 - base_lower),
                (-1.0, 1.0),
                (-1.0, 1.0),
                (-1.0, 1.0),
            ]
        )
    dimension = 4 * unit_count
    low = np.array([bound[0] for bound in bounds])
    high = np.array([bound[1] for bound in bounds])
    span = high - low

    # 全部顶点 + 随机采样 + 零点的批量评估，再做逐通道坐标细化。
    mesh = np.meshgrid(*[np.array(bound) for bound in bounds], indexing="ij")
    vertices = np.stack([entry.reshape(-1) for entry in mesh], axis=1)
    rng = np.random.default_rng(20260806)
    random_points = rng.uniform(low, high, size=(4096, dimension))
    candidates = np.vstack((np.zeros((1, dimension)), vertices, random_points))
    sampled = moment_batch(candidates)

    def refine(direction: np.ndarray) -> tuple[float, np.ndarray]:
        """沿给定方向最大化 ``direction . M`` 并做逐通道坐标细化。"""

        scores = sampled @ direction
        index = int(np.argmax(scores))
        point = candidates[index].copy()
        value = float(scores[index])
        for step in (0.25, 0.10, 0.04, 0.015, 0.006):
            improve = True
            while improve:
                improve = False
                probes = [point]
                for channel in range(dimension):
                    for sign in (1.0, -1.0):
                        probe = point.copy()
                        probe[channel] = np.clip(
                            probe[channel] + sign * step * span[channel],
                            low[channel],
                            high[channel],
                        )
                        probes.append(probe)
                probe_values = moment_batch(np.stack(probes))
                probe_scores = probe_values @ direction
                best_probe = int(np.argmax(probe_scores))
                if probe_scores[best_probe] > value + 1e-12:
                    point = probes[best_probe]
                    value = float(probe_scores[best_probe])
                    improve = True
        return value, point

    moment_max = np.zeros(3, dtype=np.float64)
    usage: list[dict[str, Any]] = []
    for axis in range(3):
        solves = []
        for sign in (1.0, -1.0):
            direction = np.zeros(3)
            direction[axis] = sign
            value, point = refine(direction)
            solves.append((sign, value, point))
        best = max(solves, key=lambda entry: abs(entry[1]))
        moment_max[axis] = abs(best[1])
        usage.append(
            {
                "axis": "xyz"[axis],
                "sign": best[0],
                "moment_n_m": best[1],
                "commands": np.asarray(best[2]).tolist(),
            }
        )
    support: dict[str, float] = {}
    for degrees in range(0, 360, 45):
        angle = np.deg2rad(degrees)
        direction = np.array([np.cos(angle), np.sin(angle), 0.0])
        value, _ = refine(direction)
        support[f"{degrees:03d}deg"] = float(value)
    alpha_max = np.abs(moment_max) / inertia.cpu().numpy()
    return {
        "moment_axis_max_n_m": {
            "x": float(abs(moment_max[0])),
            "y": float(abs(moment_max[1])),
            "z": float(abs(moment_max[2])),
        },
        "alpha_axis_max_rad_s2": {
            "x": float(alpha_max[0]),
            "y": float(alpha_max[1]),
            "z": float(alpha_max[2]),
        },
        # 水平面力矩包络（每 45 度一个方向），用于检查布局是否只是坐标重标。
        "support_horizontal_n_m": support,
        "usage": usage,
    }


def _initial_quaternion(
    roll_deg: float,
    pitch_deg: float,
    device: torch.device,
    dtype: torch.dtype,
    batch: int,
) -> torch.Tensor:
    roll = torch.full((batch, 1), np.deg2rad(roll_deg), device=device, dtype=dtype)
    pitch = torch.full((batch, 1), np.deg2rad(pitch_deg), device=device, dtype=dtype)
    zeros = torch.zeros(batch, 1, device=device, dtype=dtype)
    q_roll = torch.cat((torch.cos(roll / 2.0), torch.sin(roll / 2.0), zeros, zeros), dim=1)
    q_pitch = torch.cat((torch.cos(pitch / 2.0), zeros, torch.sin(pitch / 2.0), zeros), dim=1)
    return quaternion_multiply(q_roll, q_pitch)


def _unit_state(trim_motor_speed: torch.Tensor, batch: int) -> dict[str, torch.Tensor]:
    device = trim_motor_speed.device
    dtype = trim_motor_speed.dtype

    def zeros(*shape: int) -> torch.Tensor:
        return torch.zeros(batch, *shape, device=device, dtype=dtype)

    attitude = zeros(4)
    attitude[:, 0] = 1.0
    return {
        "position_n": zeros(3),
        "velocity_n": zeros(3),
        "attitude_q_wb": attitude,
        "angular_velocity_b": zeros(3),
        "motor_speed": trim_motor_speed.expand(batch, 2).clone(),
        "servo_angle": zeros(3),
        "servo_effective_pwm": zeros(3),
        "servo_command_angle": zeros(3),
        "servo_target_angle": zeros(3),
        "servo_motion_direction": zeros(3),
        "servo_backlash_remaining": zeros(3),
    }


def simulate_closed_loop(
    parameters: Mapping[str, torch.Tensor],
    layout: AssemblyLayout,
    model: Mapping[str, Any],
    initial_conditions: Sequence[tuple[float, float, float, float, float]],
    duration_s: float = 2.0,
    integral_limit: Sequence[float] = (0.30, 0.30, 0.60),
    trace_steps: int = 0,
) -> dict[str, Any]:
    """500 Hz 旋转通道烟测: 执行器非线性来自 SimEnv 内核，刚体积分为拼接体。"""

    device = parameters["body.mass"].device
    dtype = parameters["body.mass"].dtype
    unit_count = layout.offsets.shape[0]
    batch = len(initial_conditions)
    steps = int(round(duration_s / DT))
    gain = torch.tensor(model["gain"], device=device, dtype=dtype)
    inertia = torch.tensor(model["pair_inertia"], device=device, dtype=dtype)
    trim_speed = torch.tensor(
        model["trim"]["motor_speed"], device=device, dtype=dtype
    )
    trim_command = torch.tensor(
        model["trim"]["command"], device=device, dtype=dtype
    )
    base_lower = trim_command[1]
    upper_pwm = trim_command[0:1].expand(batch, 1)
    limits = torch.tensor(integral_limit, device=device, dtype=dtype)
    offsets = [
        torch.tensor(offset, device=device, dtype=dtype) for offset in layout.offsets
    ]

    kernels = []
    states = []
    for _ in range(unit_count):
        kernel = TensorDynamicsKernel(batch, device, dtype)
        kernel.refresh_parameters(parameters, DT)
        kernels.append(kernel)
        states.append(_unit_state(trim_speed, batch))
    active = torch.ones(batch, dtype=torch.bool, device=device)
    # 环境约定: 实例种子 [batch]，电机计数器 [batch, 2]；本烟测把噪声置零。
    seeds = torch.zeros(batch, dtype=torch.int64, device=device)
    counters = torch.zeros(batch, 2, dtype=torch.int64, device=device)

    roll = torch.tensor(
        [value[0] for value in initial_conditions], device=device, dtype=dtype
    )
    pitch = torch.tensor(
        [value[1] for value in initial_conditions], device=device, dtype=dtype
    )
    q = torch.empty(batch, 4, device=device, dtype=dtype)
    for index in range(batch):
        q[index] = _initial_quaternion(
            float(roll[index]), float(pitch[index]), device, dtype, 1
        )[0]
    rates = torch.tensor(
        [[value[2], value[3], value[4]] for value in initial_conditions],
        device=device,
        dtype=dtype,
    )
    integral = torch.zeros(batch, 3, device=device, dtype=dtype)
    target_q = torch.zeros(batch, 4, device=device, dtype=dtype)
    target_q[:, 0] = 1.0

    error_trace = torch.zeros(steps, batch, 2, device=device, dtype=dtype)
    rate_trace = torch.zeros(steps, batch, device=device, dtype=dtype)
    saturated_trace = torch.zeros(steps, batch, device=device, dtype=torch.bool)
    servo_peak = torch.zeros(batch, device=device, dtype=dtype)
    delta_peak = torch.zeros(batch, device=device, dtype=dtype)
    tilt_trace = torch.zeros(steps, batch, device=device, dtype=dtype)
    bounds_low = torch.tensor([0.0, -1.0, -1.0, -1.0], device=device, dtype=dtype)
    bounds_high = torch.tensor([1.0, 1.0, 1.0, 1.0], device=device, dtype=dtype)
    trace: list[list[Any]] = []

    for step in range(steps):
        error = quaternion_rotation_error(q, target_q)
        previous_integral = integral.clone()
        candidate = torch.clamp(
            integral + torch.cat((error[:, :2], rates[:, 2:3]), dim=1) * DT,
            -limits,
            limits,
        )
        integral = torch.where(active[:, None], candidate, integral)
        # 与 classical.LQR 一致: 电机状态取“相对配平目标转速”的偏差。
        motor_a = states[0]["motor_speed"] - trim_speed
        servo_a = states[0]["servo_angle"]
        if unit_count == 1:
            lqr_state = torch.cat(
                (error[:, :2], rates, motor_a, servo_a, integral), dim=1
            )
        else:
            lqr_state = torch.cat(
                (
                    error[:, :2],
                    rates,
                    motor_a,
                    states[1]["motor_speed"] - trim_speed,
                    servo_a,
                    states[1]["servo_angle"],
                    integral,
                ),
                dim=1,
            )
        delta = -(lqr_state @ gain.transpose(0, 1))
        commands = []
        saturated = torch.zeros(batch, dtype=torch.bool, device=device)
        for index in range(unit_count):
            unit_delta = delta[:, 4 * index : 4 * (index + 1)]
            unconstrained = torch.cat(
                ((base_lower + unit_delta[:, :1]), unit_delta[:, 1:]), dim=1
            )
            saturated = saturated | (
                (unconstrained < bounds_low) | (unconstrained > bounds_high)
            ).any(dim=1)
            command = torch.cat(
                (upper_pwm, unconstrained.clamp(bounds_low, bounds_high)), dim=1
            )
            commands.append(command)
            delta_peak = torch.maximum(delta_peak, unit_delta.abs().max(dim=1).values)
        # 饱和时冻结积分（回滚到本步候选更新之前的值），与 classical 一致。
        integral = torch.where((active & saturated)[:, None], previous_integral, integral)
        if trace_steps and step < trace_steps:
            trace.append(
                [
                    round(step * DT, 4),
                    float(error[0, 0]),
                    float(error[0, 1]),
                    float(rates[0, 0]),
                    float(rates[0, 1]),
                    float(rates[0, 2]),
                    *[float(value) for value in delta[0]],
                    bool(saturated[0]),
                ]
            )

        force_total = torch.zeros(batch, 3, device=device, dtype=dtype)
        moment_total = torch.zeros(batch, 3, device=device, dtype=dtype)
        for index, kernel in enumerate(kernels):
            next_state = kernel.step(
                states[index],
                parameters,
                commands[index],
                active,
                DT,
                seeds,
                counters,
            )
            states[index] = next_state
            force = next_state["force_b"]
            moment = next_state["moment_b"]
            force_total = force_total + force
            moment_total = moment_total + moment + torch.linalg.cross(
                offsets[index].expand_as(force), force
            )
            servo_peak = torch.maximum(
                servo_peak,
                next_state["servo_angle"].abs().max(dim=1).values,
            )
        gyroscopic = torch.linalg.cross(rates, inertia * rates)
        alpha = (moment_total - gyroscopic) / inertia
        rates_mid = rates + 0.5 * alpha * DT
        gyroscopic_mid = torch.linalg.cross(rates_mid, inertia * rates_mid)
        alpha_mid = (moment_total - gyroscopic_mid) / inertia
        rates = rates + alpha_mid * DT
        q = kernels[0]._integrate_quaternion(q, rates_mid, DT)

        error_trace[step] = error[:, :2]
        rate_trace[step] = torch.linalg.vector_norm(rates, dim=1)
        saturated_trace[step] = saturated
        tilt_trace[step] = torch.arccos(tilt_cosine(q).clamp(-1.0, 1.0))

    settled = (
        (error_trace.abs()[:, :, 0] <= CONVERGENCE_ROLL_PITCH_RAD)
        & (error_trace.abs()[:, :, 1] <= CONVERGENCE_ROLL_PITCH_RAD)
        & (rate_trace <= CONVERGENCE_RATE_RAD_S)
    )
    results = []
    for index in range(batch):
        column = settled[:, index]
        suffix = 0
        for value in reversed(column.tolist()):
            if not value:
                break
            suffix += 1
        settle_time = duration_s - suffix * DT
        converged = bool(suffix * DT >= CONVERGENCE_HOLD_S)
        safety = bool(
            (tilt_trace[:, index] <= SAFETY_TILT_RAD).all().item()
            and (rate_trace[:, index] <= SAFETY_RATE_RAD_S).all().item()
        )
        results.append(
            {
                "initial_condition": list(initial_conditions[index]),
                "converged": converged,
                "settle_time_s": round(float(settle_time), 4),
                "safety": safety,
                "saturation_fraction": float(saturated_trace[:, index].float().mean()),
                "peak_servo_angle_rad": float(servo_peak[index]),
                "peak_command_delta": float(delta_peak[index]),
                "max_tilt_rad": float(tilt_trace[:, index].max()),
                "final_roll_pitch_error_rad": [
                    float(error_trace[-1, index, 0]),
                    float(error_trace[-1, index, 1]),
                ],
                "final_rate_rad_s": float(rate_trace[-1, index]),
            }
        )
    return {
        "duration_s": duration_s,
        "unit_count": unit_count,
        "results": results,
        "converged_fraction": float(np.mean([r["converged"] for r in results])),
        "safety_fraction": float(np.mean([r["safety"] for r in results])),
        "mean_saturation_fraction": float(
            np.mean([r["saturation_fraction"] for r in results])
        ),
        **({"trace_first_unit": trace} if trace else {}),
    }


def _run_checks(
    parameters: Mapping[str, torch.Tensor],
    reference_gain: np.ndarray,
    single_model: Mapping[str, Any],
) -> dict[str, Any]:
    """三处交叉校验: plant 模型、SimEnv 内核、现役单机增益。"""

    device = parameters["body.mass"].device
    dtype = parameters["body.mass"].dtype
    plant = LocalPlantModel(parameters)
    trim = plant.hover_trim()
    count = int(parameters["body.mass"].shape[0])
    torch.manual_seed(7)
    speeds = (
        trim.motor_speed
        + torch.randn(count, 2, device=device, dtype=dtype) * 50.0
    )
    speeds = speeds.clamp_min(0.0)
    angles = torch.randn(count, 3, device=device, dtype=dtype) * 0.10
    force, moment = unit_force_moment(parameters, speeds, angles)
    reference = plant.wrench_from_actuators(speeds, angles)[:, 1:]
    plant_diff = float((moment - reference).abs().max().item())

    kernel = TensorDynamicsKernel(count, device, dtype)
    kernel.refresh_parameters(parameters, DT)
    total_thrust, motor_torque = TensorDynamicsKernel._motor_forces(
        speeds, parameters
    )
    components = kernel._forces_and_moments(
        parameters, total_thrust, motor_torque, angles
    )
    kernel_force_diff = float((force - components["force_b"]).abs().max().item())
    kernel_moment_diff = float((moment - components["moment_b"]).abs().max().item())

    gain_diff = float(
        np.abs(np.asarray(single_model["gain"]) - np.asarray(reference_gain)).max()
    )
    return {
        "plant_model_moment_max_diff": plant_diff,
        "kernel_force_max_diff": kernel_force_diff,
        "kernel_moment_max_diff": kernel_moment_diff,
        "single_gain_max_diff_vs_controller": gain_diff,
    }


def _load_weights(controller_config: Path) -> dict[str, Any]:
    loaded = load_controller_config(controller_config)
    params = loaded["params"] if "params" in loaded else loaded
    lqr = params["lqr"]
    return {
        "state_scales": [float(value) for value in lqr["state_scales"]],
        "integral_scales": [float(value) for value in lqr["integral_state_scales"]],
        "input_scales": [float(value) for value in lqr["input_scales"]],
        "input_weight_scale": float(lqr["input_weight_scale"]),
        "loaded": loaded,
    }


def _weights_with_overrides(
    weights: Mapping[str, Any], overrides: Mapping[str, float]
) -> dict[str, Any]:
    """按覆盖项调整权重: 偏航速率关注度、roll/pitch 允许误差、输入代价。"""

    state_scales = list(weights["state_scales"])
    if "yaw_rate_scale" in overrides:
        state_scales[4] = float(overrides["yaw_rate_scale"])
    if "roll_pitch_scale" in overrides:
        state_scales[0] = float(overrides["roll_pitch_scale"])
        state_scales[1] = float(overrides["roll_pitch_scale"])
    return {
        "state_scales": state_scales,
        "integral_scales": list(weights["integral_scales"]),
        "input_scales": list(weights["input_scales"]),
        "input_weight_scale": float(
            overrides.get("input_weight_scale", weights["input_weight_scale"])
        ),
    }


def _retune_entry(
    parameters: Mapping[str, torch.Tensor],
    sim_parameters: Mapping[str, torch.Tensor],
    weights: Mapping[str, Any],
    layout: AssemblyLayout,
    overrides: Mapping[str, float],
    initial_conditions: Sequence[tuple[float, float, float, float, float]],
    duration_s: float,
) -> dict[str, Any]:
    variant = _weights_with_overrides(weights, overrides)
    model = build_linear_model(
        parameters,
        layout,
        variant["state_scales"],
        variant["integral_scales"],
        variant["input_scales"],
        variant["input_weight_scale"],
    )
    smoke = simulate_closed_loop(
        sim_parameters, layout, model, initial_conditions, duration_s
    )
    return {
        "overrides": dict(overrides),
        "pole_radius_max": model["pole_radius_max"],
        "converged_fraction": smoke["converged_fraction"],
        "safety_fraction": smoke["safety_fraction"],
        "mean_saturation_fraction": smoke["mean_saturation_fraction"],
        "final_rates_rad_s": [
            round(result["final_rate_rad_s"], 4) for result in smoke["results"]
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--unit-config", type=Path, default=DEFAULT_UNIT_CONFIG)
    parser.add_argument("--controller-config", type=Path, default=DEFAULT_CONTROLLER_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--half-separation", type=float, nargs="+", default=[0.075, 0.10])
    parser.add_argument("--layouts", nargs="+", default=["side", "tandem"])
    parser.add_argument("--duration-s", type=float, default=2.0)
    parser.add_argument("--trace-steps", type=int, default=0)
    parser.add_argument("--sweep", action="store_true")
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()

    device = torch.device(args.device)
    dtype = torch.float64
    weights = _load_weights(args.controller_config)
    design_parameters = load_and_materialize(
        args.unit_config, 1, device, dtype
    ).parameters

    batch = len(DEFAULT_INITIAL_CONDITIONS)
    sim_parameters = dict(
        load_and_materialize(args.unit_config, batch, device, dtype).parameters
    )
    sim_parameters["motors.noise.stddev"] = torch.zeros_like(
        sim_parameters["motors.noise.stddev"]
    )

    started = time.time()
    single_model = synthesize_single_gain(
        design_parameters,
        weights["state_scales"],
        weights["integral_scales"],
        weights["input_scales"],
        weights["input_weight_scale"],
    )

    context = ControllerContext(
        batch_size=1,
        device=device,
        dtype=dtype,
        control_dt=DT,
        parameters=design_parameters,
    )
    controller = create_controller(weights["loaded"], context)
    reference_gain = controller._lqr_gain.detach().cpu().numpy()
    checks = _run_checks(sim_parameters, reference_gain, single_model)

    report: dict[str, Any] = {
        "schema_version": 1,
        "kind": "pair_assembly_feasibility_smoke",
        "unit_config": str(args.unit_config),
        "controller_config": str(args.controller_config),
        "dt": DT,
        "checks": checks,
        "single": {
            "pole_radius_max": single_model["pole_radius_max"],
            "authority": authority_limits(
                design_parameters,
                single_layout(),
                torch.tensor(
                    single_model["trim"]["command"], device=device, dtype=dtype
                ),
                torch.tensor(
                    single_model["trim"]["motor_speed"], device=device, dtype=dtype
                ),
                torch.tensor(
                    single_model["pair_inertia"], device=device, dtype=dtype
                ),
            ),
            "smoke": simulate_closed_loop(
                sim_parameters,
                single_layout(),
                single_model,
                DEFAULT_INITIAL_CONDITIONS,
                args.duration_s,
                trace_steps=args.trace_steps,
            ),
        },
        "pairs": {},
    }

    for direction in args.layouts:
        for half in args.half_separation:
            layout = pair_layout(direction, half)
            model = build_linear_model(
                design_parameters,
                layout,
                weights["state_scales"],
                weights["integral_scales"],
                weights["input_scales"],
                weights["input_weight_scale"],
            )
            inertia = torch.tensor(
                model["pair_inertia"], device=device, dtype=dtype
            )
            trimmed = authority_limits(
                design_parameters,
                layout,
                torch.tensor(model["trim"]["command"], device=device, dtype=dtype),
                torch.tensor(model["trim"]["motor_speed"], device=device, dtype=dtype),
                inertia,
            )
            smoke = simulate_closed_loop(
                sim_parameters,
                layout,
                model,
                DEFAULT_INITIAL_CONDITIONS,
                args.duration_s,
                trace_steps=args.trace_steps,
            )
            single_alpha = report["single"]["authority"]["alpha_axis_max_rad_s2"]
            report["pairs"][layout.name] = {
                "layout": {
                    "direction": direction,
                    "half_separation_m": half,
                    "offsets": layout.offsets.tolist(),
                },
                "pole_radius_max": model["pole_radius_max"],
                "pair_inertia": model["pair_inertia"],
                "authority": trimmed,
                "alpha_ratio_vs_single": {
                    axis: trimmed["alpha_axis_max_rad_s2"][axis] / single_alpha[axis]
                    for axis in ("x", "y", "z")
                },
                "smoke": smoke,
            }
            if args.sweep:
                report.setdefault("retune_sweep", {})[layout.name] = {
                    name: _retune_entry(
                        design_parameters,
                        sim_parameters,
                        weights,
                        layout,
                        overrides,
                        DEFAULT_INITIAL_CONDITIONS,
                        args.duration_s,
                    )
                    for name, overrides in RETUNE_VARIANTS
                }

    report["elapsed_s"] = round(time.time() - started, 3)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    summary_keys = ("converged_fraction", "safety_fraction", "mean_saturation_fraction")
    summary = {
        "checks": report["checks"],
        "single": {
            "pole_radius_max": report["single"]["pole_radius_max"],
            "alpha_max_rad_s2": report["single"]["authority"]["alpha_axis_max_rad_s2"],
            "smoke": {
                key: report["single"]["smoke"][key] for key in summary_keys
            },
        },
        "pairs": {
            name: {
                "pole_radius_max": data["pole_radius_max"],
                "pair_inertia": data["pair_inertia"],
                "alpha_max_rad_s2": data["authority"]["alpha_axis_max_rad_s2"],
                "alpha_ratio_vs_single": data["alpha_ratio_vs_single"],
                "smoke": {key: data["smoke"][key] for key in summary_keys},
            }
            for name, data in report["pairs"].items()
        },
        "output": str(args.output),
    }
    if "retune_sweep" in report:
        summary["retune_sweep_converged_fraction"] = {
            layout_name: {
                variant_name: data["converged_fraction"]
                for variant_name, data in variants.items()
            }
            for layout_name, variants in report["retune_sweep"].items()
        }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
