/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "FinalMotorOutputMap.hpp"

int main()
{
	ActuatorSetpoint output{};
	output.motors[0] = 1.0f;
	output.motors[1] = 0.75f;
	output.servos[0] = -0.25f;
	output.servos[1] = 0.5f;
	output.servos[2] = 1.0f;

	final_motor_output_map::apply(output);

	if (final_motor_output_map::kDefaultMotorScale != 0.5f) {
		return 1;
	}

	if (output.motors[0] != 0.5f || output.motors[1] != 0.375f) {
		return 2;
	}

	if (output.servos[0] != -0.25f || output.servos[1] != 0.5f
	    || output.servos[2] != 1.0f) {
		return 3;
	}

	ActuatorSetpoint idle{};
	final_motor_output_map::apply(idle);

	return (idle.motors[0] == 0.0f && idle.motors[1] == 0.0f) ? 0 : 4;
}
