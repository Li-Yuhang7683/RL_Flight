from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np
import torch
from scipy.linalg import expm, solve_discrete_are

from .base import FlightController
from .math import quaternion_rotation_error, tilt_cosine
from .plant import HoverTrim, LocalPlantModel
from .types import ControllerContext, ControllerOutput, ControllerReference, ControllerState


def _node(config: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = config.get(name, {})
    if not isinstance(value, Mapping):
        raise ValueError(f"controller.{name} must be a mapping")
    return value


def _number(config: Mapping[str, Any], name: str, default: float) -> float:
    value = float(config.get(name, default))
    if not math.isfinite(value):
        raise ValueError(f"controller parameter {name} must be finite")
    return value


def _vector(
    config: Mapping[str, Any],
    name: str,
    default: tuple[float, ...],
) -> tuple[float, ...]:
    raw = config.get(name, default)
    if not isinstance(raw, (list, tuple)) or len(raw) != len(default):
        raise ValueError(f"controller parameter {name} must have length {len(default)}")
    values = tuple(float(value) for value in raw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"controller parameter {name} must contain finite values")
    return values


class CoaxialCollectiveServoController(FlightController):
    """Motor collective/yaw plus servo-only roll/pitch control.

    Physical command contract remains SimEnv's ``[B, 5]``:
    ``[upper_motor, lower_motor, servo_1, servo_2, servo_3]``.

    The pilot collective command is a normalized vertical-acceleration
    reference. A low-passed z-acceleration PI loop corrects the common motor
    mode. The attitude LQI produces roll/pitch/yaw moments only. A local
    model-based mixer then enforces the ownership boundary:

    * both motor commands are used for total thrust and yaw differential;
    * all three servos are used only for roll and pitch moments, solving a
      constrained local inverse that cancels their common thrust and yaw
      components.

    This controller is deliberately independent from the successfully flown
    ``lqr``/``hybrid_pid_lqr`` classes and their external-upper contract.
    """

    controller_type = "lqi_coaxial_collective_servo"

    def __init__(
        self,
        context: ControllerContext,
        config: Mapping[str, Any] | None = None,
    ) -> None:
        super().__init__(context, config)
        config = self.config
        airframe = config.get("airframe", {})
        if not isinstance(airframe, Mapping):
            raise ValueError("controller.airframe must be a mapping")
        self.airframe = dict(airframe)
        self.plant = LocalPlantModel(context.parameters)
        self.trim: HoverTrim = self.plant.hover_trim()
        self.maximum_thrust = self.plant.steady_wrench(
            self.trim.command.new_tensor([[1.0, 1.0, 0.0, 0.0, 0.0]])
            .expand(context.batch_size, -1)
        )[:, 0]
        self.hover_effectiveness = self.plant.control_effectiveness(
            self.trim.command
        )
        grid_values = _vector(
            _node(config, "allocation"),
            "motor_effectiveness_grid",
            (0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00),
        )
        if (
            any(value < 0.0 or value > 1.0 for value in grid_values)
            or any(
                right <= left
                for left, right in zip(grid_values, grid_values[1:])
            )
        ):
            raise ValueError(
                "allocation.motor_effectiveness_grid must be strictly "
                "increasing within [0,1]"
            )
        self.motor_effectiveness_points = self.trim.thrust.new_tensor(
            grid_values
        )
        motor_tables = []
        for value in grid_values:
            command = self.trim.command.clone()
            command[:, :2] = value
            command[:, 2:] = 0.0
            table = self.plant.control_effectiveness(command)
            motor_tables.append(
                torch.stack(
                    (table[:, 0, :2], table[:, 3, :2]),
                    dim=1,
                )
            )
        self.motor_effectiveness_grid = torch.stack(motor_tables, dim=1)

        collective = _node(config, "collective")
        self.pilot_minimum = _number(collective, "pilot_minimum", 0.20)
        self.pilot_maximum = _number(collective, "pilot_maximum", 0.85)
        if not 0.0 <= self.pilot_minimum < self.pilot_maximum <= 1.0:
            raise ValueError(
                "collective.pilot_minimum/pilot_maximum must satisfy "
                "0 <= minimum < maximum <= 1"
            )
        self.maximum_acceleration = _number(
            collective, "maximum_acceleration_m_s2", 5.0
        )
        self.acceleration_kp = _number(collective, "acceleration_kp", 1.0)
        self.acceleration_ki = _number(collective, "acceleration_ki", 4.0)
        self.acceleration_integral_limit = _number(
            collective, "acceleration_integral_limit_m_s2", 2.0
        )
        self.acceleration_filter_time_constant = _number(
            collective, "acceleration_filter_time_constant_s", 0.010
        )
        self.minimum_thrust_fraction = _number(
            collective, "minimum_thrust_fraction", 0.02
        )
        self.maximum_thrust_fraction = _number(
            collective, "maximum_thrust_fraction", 0.98
        )
        self.zero_throttle_threshold = _number(
            collective, "zero_throttle_threshold", 0.02
        )
        if (
            self.maximum_acceleration <= 0.0
            or self.acceleration_integral_limit < 0.0
            or self.acceleration_filter_time_constant < 0.0
            or not 0.0 <= self.minimum_thrust_fraction
            < self.maximum_thrust_fraction <= 1.0
            or not 0.0 <= self.zero_throttle_threshold <= 1.0
        ):
            raise ValueError("collective controller limits are invalid")

        allocation = _node(config, "allocation")
        self.maximum_moment = self.trim.thrust.new_tensor(
            _vector(
                allocation,
                "maximum_moment_n_m",
                (0.009, 0.009, 0.025),
            )
        )
        if bool((self.maximum_moment <= 0.0).any().item()):
            raise ValueError("allocation.maximum_moment_n_m must be positive")
        self.servo_regularization = _number(
            allocation, "servo_regularization", 1.0e-8
        )
        self.servo_effectiveness_scale = _number(
            allocation, "servo_effectiveness_scale", 1.0
        )
        self.motor_regularization = _number(
            allocation, "motor_regularization", 1.0e-10
        )
        if (
            self.servo_regularization <= 0.0
            or self.servo_effectiveness_scale <= 0.0
            or self.motor_regularization <= 0.0
        ):
            raise ValueError(
                "allocation regularization/effectiveness values must be positive"
            )

        self._lqi_gain, self._lqi_poles = self._design_lqi(_node(config, "lqi"))
        self._lqi_state_size = 8
        self._lqi_input_size = 3

        self.vertical_acceleration_filter = torch.zeros(
            context.batch_size, device=context.device, dtype=context.dtype
        )
        self.vertical_acceleration_integral = torch.zeros_like(
            self.vertical_acceleration_filter
        )
        self.attitude_integral = torch.zeros(
            context.batch_size, 3, device=context.device, dtype=context.dtype
        )
        self.last_command = self.trim.command.clone()
        self._zero_command = torch.zeros_like(self.last_command)

    def reset(self, reset_mask: torch.Tensor) -> None:
        active = self._active_mask(reset_mask)
        self.vertical_acceleration_filter.masked_fill_(active, 0.0)
        self.vertical_acceleration_integral.masked_fill_(active, 0.0)
        self.attitude_integral.masked_fill_(active[:, None], 0.0)
        self.last_command.copy_(
            torch.where(active[:, None], self.trim.command, self.last_command)
        )

    def describe(self) -> dict[str, Any]:
        result = super().describe()
        result.update(
            {
                "control_allocation": {
                    "motors": "collective_common_and_yaw_differential",
                    "servos": "roll_pitch_only",
                    "vertical_feedback": "low_pass_z_acceleration_pi",
                },
                "pilot_range": [self.pilot_minimum, self.pilot_maximum],
                "airframe": self.airframe,
                "trim_command": self.trim.command[0].detach().cpu().tolist(),
                "trim_motor_speed_rad_s": (
                    self.trim.motor_speed[0].detach().cpu().tolist()
                ),
                "lqi_closed_loop_pole_radius": float(
                    np.max(np.abs(self._lqi_poles))
                ),
            }
        )
        return result

    def schedule_lqi_gain(self, gain: torch.Tensor) -> None:
        expected_shared = (self._lqi_input_size, self._lqi_state_size)
        expected_batched = (
            self.context.batch_size,
            self._lqi_input_size,
            self._lqi_state_size,
        )
        if tuple(gain.shape) not in {expected_shared, expected_batched}:
            raise ValueError(
                f"LQI gain must have shape {expected_shared} or "
                f"{expected_batched}, received {tuple(gain.shape)}"
            )
        scheduled = gain.to(device=self.context.device, dtype=self.context.dtype)
        if not bool(torch.isfinite(scheduled).all().item()):
            raise ValueError("LQI gain must contain only finite values")
        self._lqi_gain = scheduled.detach().clone()

    def _design_lqi(
        self,
        config: Mapping[str, Any],
    ) -> tuple[torch.Tensor, np.ndarray]:
        """Design the roll/pitch/yaw LQI from the airframe-1 inertia.

        State: ``[roll_error, pitch_error, p_error, q_error, r_error,
        integral_roll_error, integral_pitch_error, integral_yaw_rate_error]``.
        Input: ``[roll_moment, pitch_moment, yaw_moment]``. The local model
        intentionally excludes motor speed as an attitude input.
        """

        inertia = (
            self.context.parameters["body.inertia_diagonal_b"][0]
            .detach()
            .to(device="cpu", dtype=torch.float64)
            .numpy()
            .copy()
        )
        if np.any(inertia <= 0.0):
            raise ValueError("body inertia diagonal must be positive")

        a = np.zeros((8, 8), dtype=np.float64)
        b = np.zeros((8, 3), dtype=np.float64)
        a[0, 2] = 1.0
        a[1, 3] = 1.0
        b[2, 0] = 1.0 / inertia[0]
        b[3, 1] = 1.0 / inertia[1]
        b[4, 2] = 1.0 / inertia[2]
        a[5, 0] = 1.0
        a[6, 1] = 1.0
        a[7, 4] = 1.0

        augmented = np.block(
            [
                [a, b],
                [np.zeros((3, a.shape[0] + b.shape[1]), dtype=np.float64)],
            ]
        )
        discrete = expm(augmented * self.context.control_dt)
        ad = discrete[:8, :8]
        bd = discrete[:8, 8:]

        state_scales = np.asarray(
            _vector(
                config,
                "state_scales",
                (
                    math.radians(10.0),
                    math.radians(10.0),
                    0.7,
                    0.7,
                    1.5,
                    0.10,
                    0.10,
                    0.40,
                ),
            ),
            dtype=np.float64,
        )
        input_scales = np.asarray(
            _vector(
                config,
                "input_scales",
                (0.008, 0.008, 0.020),
            ),
            dtype=np.float64,
        )
        if np.any(state_scales <= 0.0) or np.any(input_scales <= 0.0):
            raise ValueError("LQI state and input scales must be positive")
        input_weight_scale = _number(config, "input_weight_scale", 0.15)
        if input_weight_scale <= 0.0:
            raise ValueError("lqi.input_weight_scale must be positive")

        q = np.diag(1.0 / state_scales**2)
        r = input_weight_scale * np.diag(1.0 / input_scales**2)
        solution = solve_discrete_are(ad, bd, q, r)
        gain = np.linalg.solve(
            r + bd.T @ solution @ bd,
            bd.T @ solution @ ad,
        )
        poles = np.linalg.eigvals(ad - bd @ gain)
        if np.max(np.abs(poles)) >= 1.0:
            raise ValueError("synthesized coaxial collective-servo LQI is unstable")
        return (
            torch.tensor(
                gain,
                device=self.context.device,
                dtype=self.context.dtype,
            ),
            poles,
        )

    def _allocation_jacobians(
        self,
        motor_command: torch.Tensor,
        desired_thrust: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Return local motor ``[T,Mz]`` and scaled servo Jacobians.

        Motor thrust/reaction derivatives are interpolated from a calibrated
        local table built once per controller. Grid moments scale with the
        target coaxial thrust. Both mappings are valid around the identified
        hover envelope and avoid finite-difference wrench evaluations in the
        2 ms control loop.
        """

        average_command = motor_command.mean(dim=1).clamp(
            self.motor_effectiveness_points[0],
            self.motor_effectiveness_points[-1],
        )
        upper_index = torch.searchsorted(
            self.motor_effectiveness_points,
            average_command.contiguous(),
        ).clamp(1, len(self.motor_effectiveness_points) - 1)
        lower_index = upper_index - 1
        gather_shape = (
            self.context.batch_size,
            1,
            2,
            2,
        )
        lower_table = torch.gather(
            self.motor_effectiveness_grid,
            1,
            lower_index[:, None, None, None].expand(gather_shape),
        ).squeeze(1)
        upper_table = torch.gather(
            self.motor_effectiveness_grid,
            1,
            upper_index[:, None, None, None].expand(gather_shape),
        ).squeeze(1)
        lower_command = self.motor_effectiveness_points[lower_index]
        upper_command = self.motor_effectiveness_points[upper_index]
        fraction = (
            (average_command - lower_command)
            / (upper_command - lower_command)
        )[:, None, None]
        motor_effectiveness = (
            lower_table + fraction * (upper_table - lower_table)
        )
        servo_scale = (
            desired_thrust / self.trim.thrust
        ).clamp(0.05, 1.5)
        servo_jacobian = (
            self.hover_effectiveness[:, :, 2:]
            * servo_scale[:, None, None]
            * self.servo_effectiveness_scale
        )
        return motor_effectiveness, servo_jacobian

    def _motor_mix(
        self,
        motor_effectiveness: torch.Tensor,
        yaw_moment: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Return physical motor deltas for zero thrust error and yaw demand."""

        common_differential_basis = motor_effectiveness.new_tensor(
            [[1.0, 1.0], [1.0, -1.0]]
        )
        virtual_jacobian = torch.bmm(
            motor_effectiveness,
            common_differential_basis[None].expand(
                self.context.batch_size, -1, -1
            ),
        )
        selected = virtual_jacobian + self.motor_regularization * torch.eye(
            2, device=self.context.device, dtype=self.context.dtype
        )[None]
        demand = torch.stack(
            (
                torch.zeros_like(yaw_moment),
                yaw_moment,
            ),
            dim=1,
        )
        common_differential = torch.linalg.solve(
            selected, demand.unsqueeze(-1)
        ).squeeze(-1)
        motor_delta = torch.bmm(
            common_differential_basis[None].expand(
                self.context.batch_size, -1, -1
            ),
            common_differential.unsqueeze(-1),
        ).squeeze(-1)
        return motor_delta, selected

    def _servo_mix(
        self,
        servo_jacobian: torch.Tensor,
        roll_moment: torch.Tensor,
        pitch_moment: torch.Tensor,
    ) -> tuple[torch.Tensor, torch.Tensor]:
        """Return servo deltas for roll/pitch with zero thrust/yaw output."""

        normal = torch.bmm(
            servo_jacobian.transpose(1, 2),
            servo_jacobian,
        )
        normal = normal + self.servo_regularization * torch.eye(
            3, device=self.context.device, dtype=self.context.dtype
        )[None]
        desired_servo_wrench = torch.stack(
            (
                torch.zeros_like(roll_moment),
                roll_moment,
                pitch_moment,
                torch.zeros_like(roll_moment),
            ),
            dim=1,
        )
        right_hand_side = torch.bmm(
            servo_jacobian.transpose(1, 2),
            desired_servo_wrench.unsqueeze(-1),
        )
        servo_delta = torch.linalg.solve(
            normal, right_hand_side
        ).squeeze(-1)
        achieved_servo_wrench = torch.bmm(
            servo_jacobian, servo_delta.unsqueeze(-1)
        ).squeeze(-1)
        return servo_delta, achieved_servo_wrench

    def step(
        self,
        state: ControllerState,
        reference: ControllerReference,
        active_mask: torch.Tensor | None = None,
    ) -> ControllerOutput:
        active = self._active_mask(active_mask)
        if reference.collective_command.ndim != 2 or (
            reference.collective_command.shape[1] != 1
        ):
            raise ValueError("reference.collective_command must have shape [B,1]")

        collective = reference.collective_command[:, 0].clamp(0.0, 1.0)
        zero_throttle = active & (
            collective <= self.zero_throttle_threshold
        )
        control_active = active & ~zero_throttle
        pilot_normalized = 2.0 * (
            (collective - self.pilot_minimum)
            / (self.pilot_maximum - self.pilot_minimum)
        ).clamp(0.0, 1.0) - 1.0
        target_acceleration = (
            pilot_normalized * self.maximum_acceleration
        )

        measured_acceleration = -state.linear_acceleration_n[:, 2]
        if self.acceleration_filter_time_constant > 0.0:
            alpha = self.context.control_dt / (
                self.acceleration_filter_time_constant
                + self.context.control_dt
            )
        else:
            alpha = 1.0
        filtered_acceleration = (
            self.vertical_acceleration_filter
            + alpha
            * (measured_acceleration - self.vertical_acceleration_filter)
        )
        self.vertical_acceleration_filter.copy_(
            torch.where(
                control_active,
                filtered_acceleration,
                self.vertical_acceleration_filter,
            )
        )

        acceleration_error = (
            target_acceleration - self.vertical_acceleration_filter
        )
        previous_vertical_integral = (
            self.vertical_acceleration_integral.clone()
        )
        candidate_vertical_integral = (
            previous_vertical_integral
            + acceleration_error * self.context.control_dt
        ).clamp(
            -self.acceleration_integral_limit,
            self.acceleration_integral_limit,
        )
        self.vertical_acceleration_integral.copy_(
            torch.where(
                control_active,
                candidate_vertical_integral,
                previous_vertical_integral,
            )
        )
        self.vertical_acceleration_integral.masked_fill_(zero_throttle, 0.0)
        corrected_acceleration = (
            target_acceleration
            + self.acceleration_kp * acceleration_error
            + self.acceleration_ki
            * self.vertical_acceleration_integral
        )
        acceleration_limit = (
            1.5 * self.maximum_acceleration
            + self.acceleration_integral_limit
        )
        corrected_acceleration = corrected_acceleration.clamp(
            -self.plant.gravity - acceleration_limit,
            acceleration_limit,
        )
        cosine = tilt_cosine(state.attitude_q_wb).clamp_min(0.35)
        mass = self.context.parameters["body.mass"]
        desired_thrust = (
            mass
            * (self.plant.gravity + corrected_acceleration)
            / cosine
        )
        desired_thrust = desired_thrust.clamp(
            self.minimum_thrust_fraction * self.maximum_thrust,
            self.maximum_thrust_fraction * self.maximum_thrust,
        )
        target_speed, motor_base = self.plant.balanced_motor_target(
            desired_thrust
        )

        attitude_error_full = quaternion_rotation_error(
            state.attitude_q_wb, reference.target_attitude_q_wb
        )
        attitude_error = attitude_error_full[:, :2]
        rate_error = (
            state.angular_velocity_b
            - reference.target_angular_velocity_b
        )
        previous_attitude_integral = self.attitude_integral.clone()
        candidate_attitude_integral = (
            previous_attitude_integral
            + torch.cat(
                (
                    attitude_error,
                    rate_error[:, 2:3],
                ),
                dim=1,
            )
            * self.context.control_dt
        )
        self.attitude_integral.copy_(
            torch.where(
                control_active[:, None],
                candidate_attitude_integral,
                previous_attitude_integral,
            )
        )
        self.attitude_integral.masked_fill_(zero_throttle[:, None], 0.0)
        lqi_state = torch.cat(
            (
                attitude_error,
                rate_error,
                self.attitude_integral,
            ),
            dim=1,
        )
        if self._lqi_gain.ndim == 2:
            desired_moment = -(
                lqi_state @ self._lqi_gain.transpose(0, 1)
            )
        else:
            desired_moment = -torch.bmm(
                self._lqi_gain, lqi_state.unsqueeze(-1)
            ).squeeze(-1)
        desired_moment = desired_moment.clamp(
            -self.maximum_moment, self.maximum_moment
        )

        base = torch.cat(
            (
                motor_base,
                self.trim.command[:, 2:]
                .expand(self.context.batch_size, -1),
            ),
            dim=1,
        )
        motor_effectiveness, servo_jacobian = (
            self._allocation_jacobians(
                base[:, :2], desired_thrust
            )
        )
        motor_delta, motor_singularity = self._motor_mix(
            motor_effectiveness, desired_moment[:, 2]
        )
        servo_delta, achieved_servo_wrench = self._servo_mix(
            servo_jacobian,
            desired_moment[:, 0],
            desired_moment[:, 1],
        )
        requested_servo = base[:, 2:] + servo_delta
        servo_limit = requested_servo.abs().amax(dim=1)
        servo_scale = torch.where(
            servo_limit > 1.0,
            servo_limit.reciprocal(),
            torch.ones_like(servo_limit),
        )
        # Scale the complete three-servo vector to preserve its null-yaw
        # direction. Per-channel clipping would break the zero-sum common
        # mode and create yaw whenever one servo saturated first.
        scaled_servo = base[:, 2:] + (
            requested_servo - base[:, 2:]
        ) * servo_scale[:, None]
        unscaled = base + torch.cat((motor_delta, servo_delta), dim=1)
        unconstrained = torch.cat(
            (unscaled[:, :2], scaled_servo), dim=1
        )
        achieved_servo_wrench = (
            achieved_servo_wrench * servo_scale[:, None]
        )
        lower = unconstrained.new_tensor([0.0, 0.0, -1.0, -1.0, -1.0])
        upper = unconstrained.new_tensor([1.0, 1.0, 1.0, 1.0, 1.0])
        command = unconstrained.clamp(lower, upper)
        motor_saturated = (
            (unconstrained[:, :2] < lower[:2])
            | (unconstrained[:, :2] > upper[:2])
        ).any(dim=1)
        servo_saturated = servo_limit > 1.0
        yaw_saturated = servo_saturated | (
            (desired_moment[:, 2].abs() >= self.maximum_moment[2])
            & (motor_saturated)
        )
        self.vertical_acceleration_integral.copy_(
            torch.where(
                (control_active & motor_saturated),
                previous_vertical_integral,
                self.vertical_acceleration_integral,
            )
        )
        attitude_saturated = servo_saturated | yaw_saturated
        self.attitude_integral.copy_(
            torch.where(
                (control_active & attitude_saturated)[:, None],
                previous_attitude_integral,
                self.attitude_integral,
            )
        )

        command = torch.where(zero_throttle[:, None], self._zero_command, command)
        command = torch.where(active[:, None], command, self.last_command)
        self.last_command.copy_(command)
        return ControllerOutput.create(
            command,
            {
                "controller.pilot_collective": collective,
                "controller.target_vertical_acceleration": target_acceleration,
                "controller.filtered_vertical_acceleration": (
                    self.vertical_acceleration_filter
                ),
                "controller.vertical_acceleration_error": acceleration_error,
                "controller.vertical_acceleration_integral": (
                    self.vertical_acceleration_integral
                ),
                "controller.desired_thrust": desired_thrust,
                "controller.target_motor_speed": target_speed,
                "controller.motor_common": motor_delta.mean(dim=1),
                "controller.motor_differential": (
                    motor_delta[:, 0] - motor_delta[:, 1]
                ),
                "controller.desired_moment_b": desired_moment,
                "controller.achieved_servo_wrench": achieved_servo_wrench,
                "controller.motor_mix": motor_singularity,
                "controller.motor_saturated": motor_saturated,
                "controller.servo_saturated": servo_saturated,
                "controller.attitude_error": attitude_error,
                "controller.rate_error": rate_error,
                "controller.command": command,
            },
        )
