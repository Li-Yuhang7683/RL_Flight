/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "../px4/src/modules/gs_link/GsLinkProtocol.hpp"

#include <cstdio>
#include <cstring>
#include <initializer_list>

int main()
{
	int failures = 0;

	const uint8_t crc_check[] {'1', '2', '3', '4', '5', '6', '7', '8', '9'};

	if (gs_link::crc16(crc_check, sizeof(crc_check)) != 0x29B1) {
		std::printf("FAILED: CRC-16/CCITT-FALSE check value\n");
		++failures;
	}

	const uint8_t sample[] {
		0xA5, 0x5A, 0x01, 0x01, 0x01, 0x00, 0x01, 0x00, 0x10, 0x00,
		0xE8, 0x03, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0xF0, 0xD8,
		0x01, 0x64,
		0x00, 0x00,
		0x97, 0xB2
	};

	if (sample[2 + 8 + 16] != 0x97 || sample[2 + 8 + 17] != 0xB2) {
		std::printf("FAILED: sample CRC placement\n");
		++failures;
	}

	if (gs_link::crc16(sample + 2, 8 + 16) != 0xB297) {
		std::printf("FAILED: sample CRC value\n");
		++failures;
	}

	if (sizeof(gs_link::TelemetryPayload) != 22) {
		std::printf("FAILED: telemetry base size\n");
		++failures;
	}

	for (int channels : {0, 8, 16, 18}) {
		const size_t payload = sizeof(gs_link::TelemetryPayload) + channels;

		if (payload > gs_link::kMaxPayload) {
			std::printf("FAILED: telemetry payload too large for %d channels\n", channels);
			++failures;
		}
	}

	if (sample[2] != gs_link::kVersion || sample[3] != static_cast<uint8_t>(gs_link::PacketClass::Data)
	    || sample[4] != static_cast<uint8_t>(gs_link::DataId::Rc)) {
		std::printf("FAILED: sample header contract\n");
		++failures;
	}

	if (failures != 0) {
		std::printf("GS-Link protocol: %d failure(s)\n", failures);
		return 1;
	}

	std::printf("GS-Link protocol: all checks passed\n");
	return 0;
}
