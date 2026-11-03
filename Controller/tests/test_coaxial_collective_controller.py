from __future__ import annotations

import math
import unittest
from pathlib import Path

import torch

from flight_controller import (
    ControllerContext,
    ControllerReference,
    ControllerState,
    controller_types,
    create_controller,
    load_controller_config,
)
from simenv.config import load_and_materialize


ROOT = Path(__file__).resolve().parents[2]
SIM_CONFIG = (
    ROOT
    / "SimEnv"
    / "configs"
    / "sim2real_micro_coaxial_3100kv_airframe1.yaml"
)
CONTROLLER_CONFIG = (
    ROOT
    / "Controller"
    / "configs"
    / "lqi_coaxial_collective_servo_airframe1.yaml"
)
MANUAL_SIM_CONFIG = (
    ROOT
    / "SimEnv"
    / "configs"
    / "sim2real_micro_coaxial_3100kv_airframe1_manual_v2.yaml"
)
MANUAL_CONTROLLER_CONFIG = (
    ROOT
    / "Controller"
    / "configs"
    / "lqi_coaxial_collective_servo_airframe1_manual_v2.yaml"
)
MANUAL_V3_CONTROLLER_CONFIG = (
    ROOT
    / "Controller"
    / "configs"
    / "lqi_coaxial_collective_servo_airframe1_manual_v3.yaml"
)
PILOT_CENTER = 0.5


class CoaxialCollectiveServoControllerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.materialized = load_and_materialize(
            SIM_CONFIG,
            2,
            torch.device("cpu"),
            torch.float64,
        )
        cls.context = ControllerContext(
            batch_size=2,
            device=torch.device("cpu"),
            dtype=torch.float64,
            control_dt=1.0 / 500.0,
            parameters=cls.materialized.parameters,
        )

    def create(self):
        return create_controller(
            load_controller_config(CONTROLLER_CONFIG),
            self.context,
        )

    def state_at_trim(self, controller, *, roll_error_rad: float = 0.0):
        zeros = torch.zeros(2, 3, dtype=torch.float64)
        attitude = torch.zeros(2, 4, dtype=torch.float64)
        half_angle = 0.5 * roll_error_rad
        attitude[:, 0] = math.cos(half_angle)
        attitude[:, 1] = math.sin(half_angle)
        return ControllerState(
            position_n=zeros.clone(),
            velocity_n=zeros.clone(),
            attitude_q_wb=attitude,
            angular_velocity_b=zeros.clone(),
            linear_acceleration_n=zeros.clone(),
            motor_speed=controller.trim.motor_speed.clone(),
            servo_angle=zeros.clone(),
        )

    def reference(
        self,
        *,
        yaw_rate: float = 0.0,
        collective: float = PILOT_CENTER,
    ) -> ControllerReference:
        zeros = torch.zeros(2, 3, dtype=torch.float64)
        attitude = torch.zeros(2, 4, dtype=torch.float64)
        attitude[:, 0] = 1.0
        target_rate = zeros.clone()
        target_rate[:, 2] = yaw_rate
        return ControllerReference(
            target_position_n=zeros.clone(),
            target_velocity_n=zeros.clone(),
            target_attitude_q_wb=attitude,
            target_angular_velocity_b=target_rate,
            collective_command=torch.full(
                (2, 1), collective, dtype=torch.float64
            ),
        )

    def test_controller_is_registered_as_a_separate_type(self) -> None:
        self.assertIn(
            "lqi_coaxial_collective_servo", controller_types()
        )
        self.assertIn("lqr", controller_types())
        self.assertIn("hybrid_pid_lqr", controller_types())

    def test_airframe_one_lqi_is_stable_and_keeps_five_channel_contract(self) -> None:
        controller = self.create()
        self.assertEqual(
            controller.describe()["airframe"]["id"], "airframe-1"
        )
        self.assertEqual(
            controller.describe()["airframe"][
                "identified_lqi_checksum"
            ],
            "0x1716d847",
        )
        self.assertAlmostEqual(
            float(controller.trim.command[0, 0]), 0.43933268356, places=9
        )
        self.assertAlmostEqual(
            float(controller.trim.command[0, 1]), 0.43933268356, places=9
        )
        self.assertEqual(
            tuple(controller._lqi_gain.shape),
            (3, 8),
        )
        self.assertLess(
            max(abs(controller._lqi_poles)),
            1.0,
        )
        output = controller.step(
            self.state_at_trim(controller),
            self.reference(),
        )
        self.assertEqual(tuple(output.command.shape), (2, 5))
        self.assertTrue(torch.isfinite(output.command).all())
        torch.testing.assert_close(
            output.command[:, :2],
            controller.trim.command[:, :2]
            .expand(2, -1),
            atol=1e-8,
            rtol=0.0,
        )
        torch.testing.assert_close(
            output.command[:, 2:],
            torch.zeros(2, 3, dtype=torch.float64),
            atol=1e-7,
            rtol=0.0,
        )

    def test_manual_v2_uses_flight_derived_calibration_and_lower_gain(self) -> None:
        materialized = load_and_materialize(
            MANUAL_SIM_CONFIG,
            2,
            torch.device("cpu"),
            torch.float64,
        )
        context = ControllerContext(
            batch_size=2,
            device=torch.device("cpu"),
            dtype=torch.float64,
            control_dt=1.0 / 500.0,
            parameters=materialized.parameters,
        )
        baseline = self.create()
        candidate = create_controller(
            load_controller_config(MANUAL_CONTROLLER_CONFIG),
            context,
        )
        self.assertAlmostEqual(
            float(materialized.parameters["body.mass"][0].item()),
            0.66,
        )
        torch.testing.assert_close(
            materialized.parameters["body.inertia_diagonal_b"][0],
            torch.tensor(
                [0.00050, 0.00050, 0.0007333333],
                dtype=torch.float64,
            ),
        )
        self.assertAlmostEqual(
            float(candidate.trim.command[0, 0].item()),
            0.62682926837,
            places=9,
        )
        self.assertLess(
            abs(float(candidate._lqi_gain[0, 0].item())),
            abs(float(baseline._lqi_gain[0, 0].item())),
        )
        self.assertLess(max(abs(candidate._lqi_poles)), 1.0)

    def test_manual_v3_preserves_validated_inner_loop_gains(self) -> None:
        materialized = load_and_materialize(
            MANUAL_SIM_CONFIG,
            2,
            torch.device("cpu"),
            torch.float64,
        )
        context = ControllerContext(
            batch_size=2,
            device=torch.device("cpu"),
            dtype=torch.float64,
            control_dt=1.0 / 500.0,
            parameters=materialized.parameters,
        )
        baseline = create_controller(
            load_controller_config(MANUAL_CONTROLLER_CONFIG),
            context,
        )
        candidate = create_controller(
            load_controller_config(MANUAL_V3_CONTROLLER_CONFIG),
            context,
        )
        torch.testing.assert_close(
            candidate._lqi_gain,
            baseline._lqi_gain,
            atol=1e-12,
            rtol=0.0,
        )
        self.assertLess(max(abs(candidate._lqi_poles)), 1.0)

    def test_yaw_rate_uses_motor_differential_without_servo_yaw(self) -> None:
        controller = self.create()
        output = controller.step(
            self.state_at_trim(controller),
            self.reference(yaw_rate=1.0),
        )
        self.assertGreater(
            float(
                output.diagnostics[
                    "controller.motor_differential"
                ].min()
            ),
            0.10,
        )
        self.assertTrue(
            (
                output.command[:, 0]
                > output.command[:, 1]
            ).all()
        )
        self.assertLess(
            float(output.command[:, 2:].abs().max()),
            1e-4,
        )
        self.assertLess(
            float(
                output.diagnostics[
                    "controller.achieved_servo_wrench"
                ][:, 3].abs().max()
            ),
            1e-8,
        )

    def test_roll_error_uses_servos_while_motors_stay_balanced(self) -> None:
        controller = self.create()
        output = controller.step(
            self.state_at_trim(
                controller, roll_error_rad=math.radians(5.0)
            ),
            self.reference(),
        )
        self.assertGreater(
            float(output.command[:, 2:].abs().max()),
            0.10,
        )
        torch.testing.assert_close(
            output.command[:, 2:].sum(dim=1),
            torch.zeros(2, dtype=torch.float64),
            atol=1e-7,
            rtol=0.0,
        )
        torch.testing.assert_close(
            output.diagnostics["controller.motor_differential"],
            torch.zeros(2, dtype=torch.float64),
            atol=1e-6,
            rtol=0.0,
        )
        servo_wrench = output.diagnostics[
            "controller.achieved_servo_wrench"
        ]
        self.assertGreater(
            float(servo_wrench[:, 1].abs().min()),
            1e-4,
        )
        self.assertLess(
            float(servo_wrench[:, 0].abs().max()),
            1e-10,
        )
        self.assertLess(
            float(servo_wrench[:, 3].abs().max()),
            1e-10,
        )

    def test_z_acceleration_error_corrects_common_motor_mode(self) -> None:
        controller = self.create()
        reference = self.reference(collective=0.70)
        slow_vehicle = self.state_at_trim(controller)
        slow_vehicle.linear_acceleration_n[:, 2] = 0.0
        slow_output = controller.step(slow_vehicle, reference)
        torch.testing.assert_close(
            slow_output.diagnostics["controller.motor_differential"],
            torch.zeros(2, dtype=torch.float64),
            atol=2e-6,
            rtol=0.0,
        )
        self.assertGreater(
            float(slow_output.command[0, :2].mean()),
            float(controller.trim.command[0, :2].mean()),
        )
        self.assertGreater(
            float(
                slow_output.diagnostics[
                    "controller.vertical_acceleration_error"
                ][0]
            ),
            0.0,
        )

        fast_controller = self.create()
        fast_controller.vertical_acceleration_filter.fill_(10.0)
        fast_vehicle = self.state_at_trim(fast_controller)
        fast_vehicle.linear_acceleration_n[:, 2] = -10.0
        fast_output = fast_controller.step(fast_vehicle, reference)
        self.assertLess(
            float(fast_output.command[0, :2].mean()),
            float(controller.trim.command[0, :2].mean()),
        )

    def test_zero_throttle_is_exact_off_and_clears_integrators(self) -> None:
        controller = self.create()
        controller.vertical_acceleration_integral.fill_(1.0)
        controller.attitude_integral.fill_(1.0)
        output = controller.step(
            self.state_at_trim(controller),
            self.reference(collective=0.0),
        )
        torch.testing.assert_close(
            output.command,
            torch.zeros(2, 5, dtype=torch.float64),
        )
        torch.testing.assert_close(
            controller.vertical_acceleration_integral,
            torch.zeros(2, dtype=torch.float64),
        )
        torch.testing.assert_close(
            controller.attitude_integral,
            torch.zeros(2, 3, dtype=torch.float64),
        )

    def test_masked_reset_preserves_unselected_vehicle(self) -> None:
        controller = self.create()
        controller.vertical_acceleration_integral.copy_(
            torch.tensor([1.0, 2.0], dtype=torch.float64)
        )
        controller.attitude_integral.copy_(
            torch.tensor(
                [[1.0, 1.0, 1.0], [2.0, 2.0, 2.0]],
                dtype=torch.float64,
            )
        )
        controller.reset(torch.tensor([True, False]))
        self.assertEqual(
            controller.vertical_acceleration_integral.tolist(),
            [0.0, 2.0],
        )
        torch.testing.assert_close(
            controller.attitude_integral[0],
            torch.zeros(3, dtype=torch.float64),
        )
        torch.testing.assert_close(
            controller.attitude_integral[1],
            torch.full((3,), 2.0, dtype=torch.float64),
        )


if __name__ == "__main__":
    unittest.main()
