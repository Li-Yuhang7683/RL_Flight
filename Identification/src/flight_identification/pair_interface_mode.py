"""双机体对接面载荷分析与最小载荷控制模态。

此前 ``pair_assembly`` 只把拼接体当作一个刚体，检查合运动（净力/净力矩），
**没有计算两机之间的对接面约束力/力矩**。本模块补上自由体分析：

对单元 B（与 A 关于对接面中心对称）在悬停点线性化:

    f = (F_A - F_B)/2 + m (alpha x r_B)
    c = I_unit alpha - M_B - (r_J - r_B) x f
    alpha = I_pair^{-1} (M_A + M_B + r_A x F_A + r_B x F_B)

其中 ``(f, c)`` 是 A 通过对接面作用在 B 上的合力/合力矩（关于对接点 J），
``F_i``/``M_i`` 是第 i 台机执行器相对悬停配平的扰动量。

在此基础上:

1. 构造 6x8 对接面载荷映射 L6（命令 -> f,c）与 4x8 净控制映射 B4
   （[Mx, My, Mz, Fz]）。
2. 求 L6 的零空间（接口零载荷命令子空间）以及它可覆盖的控制方向；
3. 用线性规划求各轴零载荷最大权限；
4. 偏航没有零载荷解，给出零力 / 零力矩两个 Pareto 端点及其最大权限；
5. 用真实非线性力/力矩映射复核大指令下的残余载荷。

所有结果都是标称机体的 design_only 分析，不含结构柔性、装配误差、两机气动
互扰与动态冲击。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import torch
from scipy.optimize import linprog

from flight_controller.plant import LocalPlantModel
from flight_controller.math import lookup
from simenv.config import load_and_materialize

from .pair_assembly import (
    DEFAULT_UNIT_CONFIG,
    AssemblyLayout,
    pair_inertia,
    pair_layout,
    unit_force_moment,
)


DEFAULT_OUTPUT = Path("Identification/runs/pair_interface_mode_v1/analysis.json")


def _cross_matrix(vector: torch.Tensor) -> torch.Tensor:
    x, y, z = vector.tolist()
    return torch.tensor(
        [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]],
        device=vector.device,
        dtype=vector.dtype,
    )


def unit_command_map(
    parameters: Mapping[str, torch.Tensor],
) -> tuple[torch.Tensor, Any]:
    """6x4 单机命令->[F, M] 线性映射（命令 = lower, servo_1..3）。"""

    plant = LocalPlantModel(parameters)
    trim = plant.hover_trim()
    device = parameters["body.mass"].device
    dtype = parameters["body.mass"].dtype
    state0 = torch.cat(
        (trim.motor_speed[0], torch.zeros(3, device=device, dtype=dtype))
    )

    def wrench(state: torch.Tensor) -> torch.Tensor:
        force, moment = unit_force_moment(
            parameters, state[:2][None], state[2:][None]
        )
        return torch.cat((force[0], moment[0]))

    jacobian = torch.autograd.functional.jacobian(wrench, state0)

    # 转速表: 下桨目标转速对下桨 pwm 的斜率。
    def lower_target(pwm: torch.Tensor) -> torch.Tensor:
        speed = lookup(pwm[None], parameters["motors.pwm_to_rpm_table"])[0]
        return speed[1]

    lower_slope = torch.autograd.functional.jacobian(
        lower_target, trim.command[0, 1]
    )

    def servo_target(pwm: torch.Tensor) -> torch.Tensor:
        return lookup(pwm[None], parameters["servos.pwm_angle_table"])[0]

    servo_slopes = torch.autograd.functional.jacobian(
        servo_target, torch.zeros(3, device=device, dtype=dtype)
    )
    columns = [
        jacobian[:, 1] * lower_slope,
        jacobian[:, 2] * servo_slopes[0, 0],
        jacobian[:, 3] * servo_slopes[1, 1],
        jacobian[:, 4] * servo_slopes[2, 2],
    ]
    return torch.stack(columns, dim=1), trim


def assembly_maps(
    parameters: Mapping[str, torch.Tensor],
    layout: AssemblyLayout,
) -> dict[str, Any]:
    """构造 B6 / B4 / L6 及各单元位置、惯量。"""

    device = parameters["body.mass"].device
    dtype = parameters["body.mass"].dtype
    unit_map, trim = unit_command_map(parameters)
    mass = float(parameters["body.mass"][0])
    unit_inertia = parameters["body.inertia_diagonal_b"][0]
    inertia = pair_inertia(
        parameters["body.inertia_diagonal_b"],
        parameters["body.mass"],
        layout.offsets,
        device,
        dtype,
    )
    r_a = torch.tensor(layout.offsets[0], device=device, dtype=dtype)
    r_b = torch.tensor(layout.offsets[1], device=device, dtype=dtype)
    zeros = torch.zeros(3, 4, device=device, dtype=dtype)

    def pack(r: torch.Tensor) -> torch.Tensor:
        return torch.cat(
            (unit_map[:3], unit_map[3:] + _cross_matrix(r) @ unit_map[:3]),
            dim=0,
        )

    b6 = torch.cat((pack(r_a), pack(r_b)), dim=1)
    b4 = b6[[3, 4, 5, 2]]
    f_a = torch.cat((b6[:3, :4], zeros), dim=1)
    f_b = torch.cat((zeros, b6[:3, 4:]), dim=1)
    m_b = torch.cat((zeros, unit_map[3:]), dim=1)
    alpha = (1.0 / inertia)[:, None] * b6[3:, :]
    force = (f_a - f_b) / 2.0 + mass * (-_cross_matrix(r_b) @ alpha)
    couple = (
        unit_inertia[:, None] * alpha
        - m_b
        - _cross_matrix(-r_b) @ force
    )
    load = torch.cat((force, couple), dim=0)
    return {
        "unit_map": unit_map,
        "trim": trim,
        "b6": b6,
        "b4": b4,
        "load": load,
        "inertia": inertia,
        "mass": mass,
        "r_a": r_a,
        "r_b": r_b,
        "base_lower": float(trim.command[0, 1]),
    }


def nonlinear_interface(
    commands: np.ndarray,
    parameters: Mapping[str, torch.Tensor],
    layout: AssemblyLayout,
    maps: Mapping[str, Any],
) -> dict[str, np.ndarray]:
    """用真实非线性映射计算一条 8 通道命令的对接面载荷。"""

    device = parameters["body.mass"].device
    dtype = parameters["body.mass"].dtype
    trim = maps["trim"]
    inertia = maps["inertia"]
    mass = maps["mass"]
    r_a, r_b = maps["r_a"], maps["r_b"]
    values = torch.tensor(commands, device=device, dtype=dtype)
    wrenches = []
    for index in range(2):
        lower_pwm = torch.clamp(
            trim.command[0, 1] + values[4 * index], 0.0, 1.0
        )
        motor_pwm = torch.stack((trim.command[0, 0], lower_pwm))
        speed = lookup(motor_pwm[None], parameters["motors.pwm_to_rpm_table"])[0]
        speed = torch.where(
            motor_pwm <= parameters["motors.pwm_deadzone"][0],
            torch.zeros_like(speed),
            speed,
        )
        angles = lookup(
            values[4 * index + 1 : 4 * index + 4][None],
            parameters["servos.pwm_angle_table"],
        )[0]
        force, moment = unit_force_moment(
            parameters, speed[None], angles[None]
        )
        wrenches.append((force[0], moment[0]))
    (f_a, m_a), (f_b, m_b) = wrenches
    pair_force = f_a + f_b
    pair_moment = (
        m_a
        + m_b
        + torch.linalg.cross(r_a, f_a)
        + torch.linalg.cross(r_b, f_b)
    )
    alpha = pair_moment / inertia
    force = (f_a - f_b) / 2.0 + mass * torch.linalg.cross(alpha, r_b)
    couple = (
        parameters["body.inertia_diagonal_b"][0] * alpha
        - m_b
        - torch.linalg.cross(-r_b, force)
    )
    return {
        "pair_force": pair_force.detach().cpu().numpy(),
        "pair_moment": pair_moment.detach().cpu().numpy(),
        "interface_force": force.detach().cpu().numpy(),
        "interface_couple": couple.detach().cpu().numpy(),
    }


def _limits(maps: Mapping[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    base = float(maps["base_lower"])
    low = np.tile([-base, -1.0, -1.0, -1.0], 2)
    high = np.tile([1.0 - base, 1.0, 1.0, 1.0], 2)
    return low, high


def _max_scale(
    values: np.ndarray, low: np.ndarray, high: np.ndarray
) -> float:
    """沿给定命令方向、在命令盒内的最大正比例系数。"""

    ratios = []
    for value, lower, upper in zip(values, low, high):
        if abs(value) < 1e-12:
            continue
        ratios.append(upper / value if value > 0.0 else lower / value)
    return float(min(ratios)) if ratios else 0.0


def analyze_layout(
    parameters: Mapping[str, torch.Tensor],
    layout: AssemblyLayout,
) -> dict[str, Any]:
    maps = assembly_maps(parameters, layout)
    b4 = maps["b4"].cpu().numpy()
    b6 = maps["b6"].cpu().numpy()
    load = maps["load"].cpu().numpy()
    low, high = _limits(maps)

    _, singular, vt = np.linalg.svd(load, full_matrices=True)
    rank = int(np.sum(singular > singular[0] * 1e-9))
    nullspace = vt[rank:].T
    null_image = b4 @ nullspace

    # 每个控制轴的零载荷可行性（最小二乘带零载荷约束）。
    zero_load = {}
    names = ("mx", "my", "mz", "fz")
    for axis in range(4):
        target = np.zeros(4)
        target[axis] = 1.0
        solution, residual, _, _ = np.linalg.lstsq(
            np.vstack((load, b4)),
            np.concatenate((np.zeros(6), target)),
            rcond=None,
        )
        values = load @ solution
        zero_load[names[axis]] = {
            "achievable": bool(float(residual[0]) < 1e-12)
            if residual.size
            else True,
            "residual": float(residual[0]) if residual.size else 0.0,
            "interface_force_n": float(np.linalg.norm(values[:3])),
            "interface_couple_n_m": float(np.linalg.norm(values[3:])),
        }

    # 零载荷子空间的每轴最大权限（命令盒约束）。
    matrix = null_image
    zero_authority = {}
    for axis, name in ((0, "mx"), (1, "my"), (3, "fz")):
        others = [row for row in range(4) if row != axis]
        result = linprog(
            c=-matrix[axis],
            A_ub=np.vstack((nullspace, -nullspace)),
            b_ub=np.concatenate((high, -low)),
            A_eq=matrix[others],
            b_eq=np.zeros(len(others)),
            bounds=[(None, None)] * nullspace.shape[1],
            method="highs",
        )
        if not result.success:
            zero_authority[name] = None
            continue
        commands = nullspace @ result.x
        zero_authority[name] = {
            "max_axis": float(-result.fun),
            "commands": commands.tolist(),
            "command_norm": float(np.linalg.norm(commands)),
        }

    # 全命令盒下的线性权限（用于效率对照）。
    full_authority = {}
    for axis, name in ((0, "mx"), (1, "my"), (2, "mz"), (3, "fz")):
        result = linprog(
            c=-b4[axis],
            A_ub=np.vstack((np.eye(8), -np.eye(8))),
            b_ub=np.concatenate((high, -low)),
            bounds=[(None, None)] * 8,
            method="highs",
        )
        full_authority[name] = float(-result.fun) if result.success else None

    # 偏航 Pareto：零力 / 零力矩两个端点。
    yaw_modes = {}
    for name, (wf, wc) in {
        "zero_force": (1.0, 0.0),
        "zero_couple": (0.0, 1.0),
    }.items():
        weight = np.diag([wf] * 3 + [wc] * 3) + 1e-12 * np.eye(6)
        hessian = load.T @ weight @ load + 1e-12 * np.eye(8)
        target = np.zeros(4)
        target[2] = 1.0
        kkt = np.block(
            [
                [hessian, b4.T],
                [b4, np.zeros((4, 4))],
            ]
        )
        solution = np.linalg.solve(
            kkt, np.concatenate((np.zeros(8), target))
        )[:8]
        values = load @ solution
        margin = _max_scale(solution, low, high)
        yaw_modes[name] = {
            "per_unit_mz": {
                "interface_force_n": float(np.linalg.norm(values[:3])),
                "interface_couple_n_m": float(np.linalg.norm(values[3:])),
            },
            "commands": solution.tolist(),
            "max_mz": float(margin),
            "load_at_max": {
                "interface_force_n": float(margin * np.linalg.norm(values[:3])),
                "interface_couple_n_m": float(
                    margin * np.linalg.norm(values[3:])
                ),
            },
        }

    # 零载荷模态的非线性复核。
    nonlinear = {}
    for name, entry in zero_authority.items():
        if entry is None:
            continue
        direction = np.asarray(entry["commands"])
        samples = []
        for scale in (0.25, 0.5, 0.75, 1.0):
            values = nonlinear_interface(
                direction * scale, parameters, layout, maps
            )
            samples.append(
                {
                    "scale": scale,
                    "interface_force_n": float(
                        np.linalg.norm(values["interface_force"])
                    ),
                    "interface_couple_n_m": float(
                        np.linalg.norm(values["interface_couple"])
                    ),
                    "pair_moment": values["pair_moment"].tolist(),
                    "pair_force": values["pair_force"].tolist(),
                }
            )
        nonlinear[name] = samples

    # 4 轴分配基: 3 个零载荷轴 + 1 个偏航 Pareto 端点。
    load_free_columns = nullspace @ np.linalg.pinv(null_image[[0, 1, 3], :])
    yaw_column = np.asarray(yaw_modes["zero_couple"]["commands"])
    allocation = np.column_stack((load_free_columns, yaw_column))
    allocation_check = {
        "rank": int(np.linalg.matrix_rank(allocation)),
        "b4a": (b4 @ allocation).tolist(),
        "column_norms": np.linalg.norm(allocation, axis=0).tolist(),
        "max_abs_command": float(np.abs(allocation).max()),
    }

    return {
        "layout": {
            "name": layout.name,
            "offsets": layout.offsets.tolist(),
        },
        "mass_kg": maps["mass"],
        "inertia": maps["inertia"].cpu().numpy().tolist(),
        "ranks": {
            "load_map": rank,
            "load_nullspace_dim": int(nullspace.shape[1]),
            "wrench_map_b6": int(np.linalg.matrix_rank(b6)),
            "control_map_b4": int(np.linalg.matrix_rank(b4)),
            "nullspace_image_b4": int(np.linalg.matrix_rank(null_image)),
        },
        "load_singular_values": singular.tolist(),
        "zero_load_axis": zero_load,
        "zero_load_authority": zero_authority,
        "full_linear_authority": full_authority,
        "yaw_modes": yaw_modes,
        "nonlinear_check": nonlinear,
        "nullspace_image": null_image.tolist(),
        "allocation_zero_couple": allocation.tolist(),
        "allocation_check": allocation_check,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--unit-config", type=Path, default=DEFAULT_UNIT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--half-separation", type=float, nargs="+", default=[0.075, 0.10])
    parser.add_argument("--layouts", nargs="+", default=["side", "tandem"])
    args = parser.parse_args()
    device = torch.device("cpu")
    dtype = torch.float64
    parameters = load_and_materialize(
        args.unit_config, 1, device, dtype
    ).parameters
    report: dict[str, Any] = {"schema_version": 1, "kind": "pair_interface_mode", "layouts": {}}
    for direction in args.layouts:
        for half in args.half_separation:
            layout = pair_layout(direction, half)
            report["layouts"][layout.name] = analyze_layout(parameters, layout)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    summary = {}
    for name, entry in report["layouts"].items():
        summary[name] = {
            "ranks": entry["ranks"],
            "zero_load_axis": {
                key: value["achievable"]
                for key, value in entry["zero_load_axis"].items()
            },
            "zero_load_max": {
                key: None if value is None else round(value["max_axis"], 5)
                for key, value in entry["zero_load_authority"].items()
            },
            "full_linear_max": {
                key: None if value is None else round(value, 5)
                for key, value in entry["full_linear_authority"].items()
            },
            "yaw_zero_force": entry["yaw_modes"]["zero_force"]["per_unit_mz"],
            "yaw_zero_couple": entry["yaw_modes"]["zero_couple"]["per_unit_mz"],
            "allocation": entry["allocation_check"],
        }
    print(json.dumps(summary, indent=2))
    print(f"written: {args.output}")


if __name__ == "__main__":
    main()
