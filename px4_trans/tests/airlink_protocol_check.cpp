#include "AirlinkProtocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace
{

int g_failures = 0;

#define CHECK(condition) do { \
	if (!(condition)) { \
		std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
		++g_failures; \
	} \
} while (0)

std::vector<uint8_t> hex(const char *text)
{
	std::vector<uint8_t> bytes;
	int high = -1;

	for (const char *cursor = text; *cursor != '\0'; ++cursor) {
		int value = -1;

		if (*cursor >= '0' && *cursor <= '9') {
			value = *cursor - '0';

		} else if (*cursor >= 'A' && *cursor <= 'F') {
			value = *cursor - 'A' + 10;

		} else if (*cursor >= 'a' && *cursor <= 'f') {
			value = *cursor - 'a' + 10;
		}

		if (value < 0) {
			continue;
		}

		if (high < 0) {
			high = value;

		} else {
			bytes.push_back(static_cast<uint8_t>((high << 4) | value));
			high = -1;
		}
	}

	CHECK(high < 0);
	return bytes;
}

void expect_bytes(const uint8_t *actual, const std::vector<uint8_t> &expected)
{
	CHECK(actual != nullptr);

	if (!std::equal(expected.begin(), expected.end(), actual)) {
		std::fprintf(stderr, "byte mismatch: expected %zu bytes:", expected.size());

		for (size_t i = 0; i < expected.size(); ++i) {
			if (actual[i] != expected[i]) {
				std::fprintf(stderr, " offset=%zu expected=%02X actual=%02X",
					     i, expected[i], actual[i]);
				break;
			}
		}

		std::fputc('\n', stderr);
		++g_failures;
	}
}

airlink::Header header(airlink::FrameType type, uint8_t message_id, airlink::Role source,
		       uint32_t session, uint16_t sequence, uint16_t payload_len, uint8_t flags = 0)
{
	airlink::Header result{};
	result.frame_type = type;
	result.message_id = message_id;
	result.flags = flags;
	result.source_role = source;
	result.dest_role = source == airlink::Role::Master ? airlink::Role::Slave : airlink::Role::Master;
	result.source_session = session;
	result.sequence = sequence;
	result.payload_len = payload_len;
	return result;
}

airlink::SensorInput nominal_sensor_input()
{
	airlink::SensorInput input{};
	input.accel_valid = true;
	input.gyro_valid = true;
	input.attitude_valid = true;
	input.angular_velocity_valid = true;
	input.esc_upper_valid = true;
	input.esc_lower_valid = true;
	input.battery_valid = true;
	input.core_sources_fresh = true;
	input.local_rc_valid = true;
	input.safety_gate_allow = true;
	input.airlink_control_active = true;
	input.imu_sample_time_us = 1'000'000;
	input.attitude_sample_time_us = 1'002'000;
	input.accelerometer[2] = -1255.f / 128.f;
	input.attitude_q[0] = 1.f;
	input.esc_rpm[0] = 6000;
	input.esc_rpm[1] = 6000;
	input.battery_voltage_v = 14.8f;
	return input;
}

void test_crc_and_primitives()
{
	const uint8_t check[] {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
	CHECK(airlink::crc32c(check, sizeof(check)) == airlink::kCrc32cCheck);
	CHECK(airlink::round_half_away(2.5f) == 3);
	CHECK(airlink::round_half_away(-2.5f) == -3);
	CHECK(airlink::round_half_away(2.49f) == 2);

	uint32_t delta = 0;
	CHECK(airlink::u32_forward_delta(0x00000100u, 0xFFFFFF00u, delta));
	CHECK(delta == 0x200u);
	CHECK(airlink::u32_forward_delta(0u, 0xFFFFFFFFu, delta));
	CHECK(delta == 1u);
	CHECK(!airlink::u32_forward_delta(0u, 0x80000000u, delta));
}

void test_sensor_golden()
{
	const auto zero_frame = hex(
					  "A5 5A 01 01 00 01 02 01 40 30 20 10 00 00 2A 00"
					  "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
					  "00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00"
					  "00 00 00 00 00 00 00 00 00 00 11 4E F8 0C 5A A5");
	CHECK(zero_frame.size() == 64);

	airlink::StreamParser parser;
	parser.feed(zero_frame.data(), zero_frame.size(), 100);
	airlink::Frame frame{};
	CHECK(parser.next(frame));
	CHECK(frame.header.frame_type == airlink::FrameType::Data);
	CHECK(frame.header.payload_len == airlink::kSensorPayloadSize);

	airlink::SensorState zero_state{};
	CHECK(airlink::decode_sensor_state(frame.payload, frame.header.payload_len, zero_state));
	CHECK(zero_state.status_flags == 0);

	airlink::SensorInput input = nominal_sensor_input();
	airlink::SensorState state{};
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK(state.status_flags == 0x07FFu);

	uint8_t payload[airlink::kSensorPayloadSize] {};
	CHECK(airlink::encode_sensor_payload(state, payload));
	expect_bytes(payload, hex(
				      "FF 07 40 42 0F 00 10 4A 0F 00 00 00 00 00 19 FB"
				      "00 00 00 00 00 00 FF 7F 00 00 00 00 00 00 00 00"
				      "00 00 00 00 70 17 70 17 D0 39"));

	uint8_t output[airlink::kMaxFrameSize] {};
	size_t frame_len = 0;
	const airlink::Header nominal_header = header(airlink::FrameType::Data,
			static_cast<uint8_t>(airlink::MessageId::SensorState),
			airlink::Role::Slave, 0x10203040u, 42, sizeof(payload));
	CHECK(airlink::build_frame(nominal_header, payload, sizeof(payload), output, sizeof(output),
				   frame_len));
	CHECK(frame_len == 64);
	expect_bytes(output, hex(
				     "A5 5A 02 01 00 01 02 01 40 30 20 10 2A 00 2A 00"
				     "FF 07 40 42 0F 00 10 4A 0F 00 00 00 00 00 19 FB"
				     "00 00 00 00 00 00 FF 7F 00 00 00 00 00 00 00 00"
				     "00 00 00 00 70 17 70 17 D0 39 17 9B B1 A2 5A A5"));

	const auto boundary_frame = hex(
					    "A5 5A 01 01 00 01 02 01 40 30 20 10 FF FF 2A 00"
					    "FF 3F 00 FF FF FF 00 01 00 00 00 80 00 00 FF 7F"
					    "00 80 00 00 FF 7F FF 7F 00 00 00 00 00 00 00 80"
					    "00 00 FF 7F 01 80 FF 7F FF FF D0 4E 69 E5 5A A5");
	parser.feed(boundary_frame.data(), boundary_frame.size(), 200);
	CHECK(parser.next(frame));
	CHECK(airlink::decode_sensor_state(frame.payload, frame.header.payload_len, state));
	CHECK(state.status_flags == 0x3FFFu);
	CHECK(state.accelerometer[0] == INT16_MIN);
	CHECK(state.accelerometer[1] == 0);
	CHECK(state.accelerometer[2] == INT16_MAX);
	CHECK(state.gyro[0] == INT16_MIN);
	CHECK(state.gyro[2] == INT16_MAX);
	CHECK(state.attitude_q[0] == INT16_MAX);
	CHECK(state.angular_velocity[0] == INT16_MIN);
	CHECK(state.angular_velocity[2] == INT16_MAX);
	CHECK(state.esc_rpm[0] == -32767);
	CHECK(state.esc_rpm[1] == 32767);
	CHECK(state.battery_voltage_mv == UINT16_MAX);
}

void test_sensor_boundaries_and_optional_sources()
{
	airlink::SensorInput input{};
	input.accel_valid = true;
	input.accelerometer[0] = -256.f;
	input.accelerometer[1] = 255.9921875f;
	input.accelerometer[2] = 300.f;
	airlink::SensorState state{};
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK(state.accelerometer[0] == INT16_MIN);
	CHECK(state.accelerometer[1] == INT16_MAX);
	CHECK(state.accelerometer[2] == INT16_MAX);
	CHECK((state.status_flags & airlink::AccelValid) != 0);
	CHECK((state.status_flags & airlink::AccelClipped) != 0);

	input = {};
	input.gyro_valid = true;
	input.gyro[0] = -40.96f;
	input.gyro[1] = 40.95875f;
	input.gyro[2] = -50.f;
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK(state.gyro[0] == INT16_MIN);
	CHECK(state.gyro[1] == INT16_MAX);
	CHECK(state.gyro[2] == INT16_MIN);
	CHECK((state.status_flags & airlink::GyroClipped) != 0);

	input = nominal_sensor_input();
	input.esc_lower_valid = false;
	input.esc_rpm[1] = 6500;
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK((state.status_flags & airlink::EscUpperValid) != 0);
	CHECK((state.status_flags & airlink::EscLowerValid) == 0);
	CHECK(state.esc_rpm[0] == 6000);
	CHECK(state.esc_rpm[1] == 0);

	input = nominal_sensor_input();
	input.battery_voltage_v = std::numeric_limits<float>::quiet_NaN();
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK((state.status_flags & airlink::BatteryValid) == 0);
	CHECK(state.battery_voltage_mv == 0);

	input.battery_voltage_v = 65.536f;
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK((state.status_flags & airlink::BatteryValid) == 0);
	CHECK(state.battery_voltage_mv == 0);

	input.battery_voltage_v = 65.535f;
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK((state.status_flags & airlink::BatteryValid) != 0);
	CHECK(state.battery_voltage_mv == UINT16_MAX);
}

void test_yaw_reference_round_trip()
{
	airlink::YawReference reference{};
	reference.source_sample_time_us = 0x89ABCDEFu;
	reference.yaw_rad = -3.1415f;
	reference.yaw_sigma_rad = 0.1234f;
	reference.reset_counter = 7;
	reference.quality = 2;
	reference.flags = airlink::YawValid;
	uint8_t payload[airlink::kYawReferencePayloadSize] {};
	CHECK(airlink::encode_yaw_reference(reference, payload));
	expect_bytes(payload, hex(
				      "EF CD AB 89 49 85 D2 04 07 02 01 00"));

	airlink::YawReference decoded{};
	CHECK(airlink::decode_yaw_reference(payload, sizeof(payload), decoded));
	CHECK(decoded.source_sample_time_us == reference.source_sample_time_us);
	CHECK(std::fabs(decoded.yaw_rad - reference.yaw_rad) <= 1e-4f);
	CHECK(std::fabs(decoded.yaw_sigma_rad - reference.yaw_sigma_rad) <= 1e-4f);
	CHECK(decoded.reset_counter == reference.reset_counter);
	CHECK(decoded.quality == reference.quality);
	CHECK(decoded.flags == reference.flags);

	reference.flags |= airlink::ReservedYawReferenceFlags;
	CHECK(!airlink::encode_yaw_reference(reference, payload));

	airlink::Header header_value = header(airlink::FrameType::Data,
					     static_cast<uint8_t>(airlink::MessageId::YawReference),
					     airlink::Role::Master, 0x10203040u, 9,
					     airlink::kYawReferencePayloadSize);
	uint16_t minimum = 0;
	uint16_t maximum = 0;
	CHECK(airlink::expected_payload_length(header_value.frame_type,
					       header_value.message_id, minimum, maximum));
	CHECK(minimum == airlink::kYawReferencePayloadSize);
	CHECK(maximum == airlink::kYawReferencePayloadSize);
}

void test_quaternion_rules()
{
	airlink::SensorInput input{};
	input.attitude_valid = true;
	input.attitude_q[0] = 1.f;
	airlink::SensorState state{};
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK((state.status_flags & airlink::AttitudeValid) != 0);
	CHECK(state.attitude_q[0] == 32767);

	input.attitude_q[0] = -1.f;
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK((state.status_flags & airlink::AttitudeValid) != 0);
	CHECK(state.attitude_q[0] == -32767);

	input.attitude_q[0] = 0.5f;
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK((state.status_flags & airlink::AttitudeValid) == 0);
	CHECK(std::all_of(std::begin(state.attitude_q), std::end(state.attitude_q),
			  [](int16_t value) { return value == 0; }));

	input.attitude_q[0] = std::numeric_limits<float>::quiet_NaN();
	CHECK(airlink::encode_sensor_state(input, state));
	CHECK((state.status_flags & airlink::AttitudeValid) == 0);

	uint8_t payload[airlink::kSensorPayloadSize] {};
	airlink::SensorState decoded{};
	airlink::write_u16(payload, airlink::AttitudeValid);
	airlink::write_u16(payload + 22, 20000);
	CHECK(airlink::decode_sensor_state(payload, sizeof(payload), decoded));
	CHECK((decoded.status_flags & airlink::AttitudeValid) == 0);
	CHECK(decoded.attitude_q[0] == 0);
}

void test_actuator_encoding()
{
	uint8_t payload[airlink::kActuatorPayloadSize] {};
	airlink::ActuatorCommand command{};
	command.servos[0] = command.servos[1] = command.servos[2] = -1.f;
	CHECK(airlink::encode_actuator_command(command, payload));
	expect_bytes(payload, hex("00 00 00 00 00 00"));

	command.upper_motor = 0.5f;
	command.lower_motor = 0.5f;
	command.servos[0] = command.servos[1] = command.servos[2] = 0.f;
	CHECK(airlink::encode_actuator_command(command, payload));
	expect_bytes(payload, hex("FA F4 D1 47 1F 7D"));

	command.upper_motor = 1.f;
	command.lower_motor = 1.f;
	command.servos[0] = command.servos[1] = command.servos[2] = 1.f;
	CHECK(airlink::encode_actuator_command(command, payload));
	expect_bytes(payload, hex("F4 E9 A3 8F 3E FA"));

	airlink::ActuatorCommand decoded{};
	CHECK(airlink::decode_actuator_command(payload, sizeof(payload), decoded));
	CHECK(decoded.upper_motor == 1.f);
	CHECK(decoded.lower_motor == 1.f);
	CHECK(decoded.servos[0] == 1.f);
	CHECK(decoded.servos[2] == 1.f);

	command.upper_motor = std::numeric_limits<float>::infinity();
	CHECK(!airlink::encode_actuator_command(command, payload));
	command.upper_motor = 1.01f;
	CHECK(!airlink::encode_actuator_command(command, payload));

	// Invalid upper motor code 501 must reject the whole command.
	uint8_t invalid[6] {};
	airlink::encode_actuator_command(airlink::ActuatorCommand{}, invalid);
	invalid[0] = static_cast<uint8_t>(501u & 0xFFu);
	invalid[1] = static_cast<uint8_t>(501u >> 8);
	CHECK(!airlink::decode_actuator_command(invalid, sizeof(invalid), decoded));

	// Invalid servo 3 code 1001 must also reject the complete payload.
	memset(invalid, 0, sizeof(invalid));
	const uint64_t invalid_servo = static_cast<uint64_t>(1001) << 38;

	for (int i = 0; i < 6; ++i) {
		invalid[i] = static_cast<uint8_t>(invalid_servo >> (i * 8));
	}

	CHECK(!airlink::decode_actuator_command(invalid, sizeof(invalid), decoded));
}

void test_fixed_message_contracts()
{
	airlink::Ack ack{};
	ack.target_peer_session = 0x11223344u;
	ack.event_id = 0x55667788u;
	ack.acked_message_id = 0x07;
	ack.result = airlink::AckResult::Success;
	uint8_t ack_payload[airlink::kAckPayloadSize] {};
	CHECK(airlink::encode_ack(ack, ack_payload));
	airlink::Ack decoded_ack{};
	CHECK(airlink::decode_ack(ack_payload, sizeof(ack_payload), decoded_ack));
	CHECK(decoded_ack.event_id == ack.event_id);
	ack_payload[11] = 1;
	CHECK(!airlink::decode_ack(ack_payload, sizeof(ack_payload), decoded_ack));

	uint8_t handshake_payload[airlink::kHandshakePayloadSize] {};
	airlink::Handshake handshake{};
	handshake.target_peer_session = 0x10203040u;
	handshake.transaction_id = 0x11223344u;
	handshake.phase = airlink::HandshakePhase::Request;
	handshake.capabilities = 0xFFu;
	handshake.state_flags = 0xC9u;
	handshake.config_crc32c = 0x89ABCDEFu;
	CHECK(airlink::encode_handshake(handshake, handshake_payload));
	expect_bytes(handshake_payload, hex(
					     "40 30 20 10 44 33 22 11 02 00 01 00"
					     "FF 00 00 00 C9 00 00 00 EF CD AB 89"));

	uint8_t output[airlink::kMaxFrameSize] {};
	size_t frame_len = 0;
	const airlink::Header handshake_header = header(airlink::FrameType::Command,
			static_cast<uint8_t>(airlink::MessageId::LinkHandshake),
			airlink::Role::Master, 0x55667788u, 0x0101, sizeof(handshake_payload));
	CHECK(airlink::build_frame(handshake_header, handshake_payload, sizeof(handshake_payload),
				   output, sizeof(output), frame_len));
	CHECK(frame_len == 46);
	CHECK(airlink::read_u32(output + frame_len - 6) == 0x9A044D7Bu);
	expect_bytes(output, hex(
				     "A5 5A 02 02 00 09 01 02 88 77 66 55 01 01 18 00"
				     "40 30 20 10 44 33 22 11 02 00 01 00 FF 00 00 00"
				     "C9 00 00 00 EF CD AB 89 7B 4D 04 9A 5A A5"));

	airlink::ReliableEvent event{};
	event.target_peer_session = 0x10203040u;
	event.event_id = 0xA1B2C3D4u;
	event.reason_code = airlink::ReasonCode::OperatorRequest;
	uint8_t event_payload[airlink::kReliableEventPayloadSize] {};
	CHECK(airlink::encode_reliable_event(event, event_payload));
	const airlink::Header arm_header = header(airlink::FrameType::Command,
			static_cast<uint8_t>(airlink::MessageId::PairArm),
			airlink::Role::Master, 0x55667788u, 0x0100, sizeof(event_payload),
			airlink::AckRequired);
	CHECK(airlink::build_frame(arm_header, event_payload, sizeof(event_payload), output, sizeof(output),
				   frame_len));
	CHECK(frame_len == 34);
	CHECK(airlink::read_u32(output + frame_len - 6) == 0x830BC7AEu);
	expect_bytes(output, hex(
				     "A5 5A 02 02 01 07 01 02 88 77 66 55 00 01 0C 00"
				     "40 30 20 10 D4 C3 B2 A1 01 00 00 00 AE C7 0B 83"
				     "5A A5"));
}

bool parse_one(const std::vector<uint8_t> &bytes, airlink::Frame &frame)
{
	airlink::StreamParser parser;
	parser.feed(bytes.data(), bytes.size(), 10);
	return parser.next(frame);
}

void test_stream_parser()
{
	const auto nominal = hex(
				      "A5 5A 01 01 00 01 02 01 40 30 20 10 2A 00 2A 00"
				      "FF 07 40 42 0F 00 10 4A 0F 00 00 00 00 00 19 FB"
				      "00 00 00 00 00 00 FF 7F 00 00 00 00 00 00 00 00"
				      "00 00 00 00 70 17 70 17 D0 39 EE 8B 82 05 5A A5");
	airlink::Frame frame{};

	for (size_t split = 0; split <= nominal.size(); ++split) {
		airlink::StreamParser parser;
		parser.feed(nominal.data(), split, 100);
		const bool parsed_before_second_feed = parser.next(frame);
		CHECK(!parsed_before_second_feed || split == nominal.size());
		parser.feed(nominal.data() + split, nominal.size() - split, 101);
		CHECK(parsed_before_second_feed || parser.next(frame));
		CHECK(frame.header.payload_len == 42);
	}

	airlink::StreamParser parser;
	std::vector<uint8_t> concatenated {'X', 'Y', 0xA5, 0xA5, 0x5A};
	concatenated.insert(concatenated.end(), nominal.begin(), nominal.end());
	concatenated.insert(concatenated.end(), nominal.begin(), nominal.end());
	parser.feed(concatenated.data(), concatenated.size(), 200);
	CHECK(parser.next(frame));
	CHECK(parser.next(frame));
	CHECK(!parser.next(frame));

	// SOF-looking payload bytes must never split the frame.
	auto embedded_sof = nominal;
	embedded_sof[16] = 0xA5;
	embedded_sof[17] = 0x5A;
	embedded_sof[18] = 0x5A;
	embedded_sof[19] = 0xA5;
	const uint32_t new_crc = airlink::crc32c(embedded_sof.data() + 2, 16 - 2 + 42);
	airlink::write_u32(embedded_sof.data() + 16 + 42, new_crc);
	CHECK(parse_one(embedded_sof, frame));

	auto bad_crc = nominal;
	bad_crc[30] ^= 1;
	CHECK(!parse_one(bad_crc, frame));

	auto bad_eof = nominal;
	bad_eof.back() = 0;
	CHECK(!parse_one(bad_eof, frame));

	auto bad_version = nominal;
	bad_version[2] = 2;
	CHECK(!parse_one(bad_version, frame));

	auto bad_role = nominal;
	bad_role[7] = 2;
	CHECK(!parse_one(bad_role, frame));

	auto bad_reserved_flags = nominal;
	bad_reserved_flags[4] = 0x80;
	CHECK(!parse_one(bad_reserved_flags, frame));

	for (uint16_t bad_length : {static_cast<uint16_t>(41), static_cast<uint16_t>(43)}) {
		auto bad_sensor_length = nominal;
		airlink::write_u16(bad_sensor_length.data() + 14, bad_length);
		const uint32_t crc = airlink::crc32c(bad_sensor_length.data() + 2, 14 + bad_length);
		airlink::write_u32(bad_sensor_length.data() + 16 + bad_length, crc);
		airlink::StreamParser length_parser;
		length_parser.feed(bad_sensor_length.data(), bad_sensor_length.size(), 300);
		CHECK(!length_parser.next(frame));
		CHECK(length_parser.stats().length_errors > 0);
	}

	airlink::StreamParser timeout_parser;
	timeout_parser.feed(nominal.data(), 10, 1'000);
	timeout_parser.check_timeout(6'001);
	CHECK(timeout_parser.stats().partial_timeouts == 1);
	CHECK(timeout_parser.buffered() == 0);

	airlink::StreamParser bounded_parser;
	CHECK(bounded_parser.space_remaining() == airlink::kMaxFrameSize);
	const std::vector<uint8_t> oversized(airlink::kMaxFrameSize + 1, 0);
	bounded_parser.feed(oversized.data(), oversized.size(), 200);
	CHECK(bounded_parser.stats().buffer_overflows == 1);
	CHECK(bounded_parser.stats().resyncs == 0);
	CHECK(bounded_parser.buffered() == airlink::kMaxFrameSize);

	const uint8_t unknown_payload[8] {0x44, 0x33, 0x22, 0x11, 0, 0, 0, 0};
	uint8_t unknown_frame[airlink::kMaxFrameSize] {};
	size_t unknown_length = 0;
	const airlink::Header unknown_command = header(airlink::FrameType::Command, 0x20,
			airlink::Role::Master, 1, 7, sizeof(unknown_payload),
			airlink::AckRequired);
	CHECK(airlink::build_frame(unknown_command, unknown_payload, sizeof(unknown_payload),
				   unknown_frame, sizeof(unknown_frame), unknown_length));
	CHECK(parse_one(std::vector<uint8_t>(unknown_frame, unknown_frame + unknown_length), frame));
	CHECK(frame.header.message_id == 0x20);

	const airlink::Header unknown_data = header(airlink::FrameType::Data, 0x20,
			airlink::Role::Slave, 1, 8, 3);
	CHECK(airlink::build_frame(unknown_data, unknown_payload, 3, unknown_frame,
				   sizeof(unknown_frame), unknown_length));
	CHECK(parse_one(std::vector<uint8_t>(unknown_frame, unknown_frame + unknown_length), frame));

	const airlink::Header bad_periodic_flags = header(airlink::FrameType::Data,
			static_cast<uint8_t>(airlink::MessageId::SensorState),
			airlink::Role::Slave, 1, 9, 42, airlink::AckRequired);
	CHECK(!airlink::build_frame(bad_periodic_flags, nominal.data() + 16, 42,
				    unknown_frame, sizeof(unknown_frame), unknown_length));
}

void test_link_test_contracts()
{
	airlink::LinkTestControl control{};
	control.target_peer_session = 1;
	control.test_id = 2;
	control.payload_len = 256;
	control.rate_hz = 500;
	control.duration_ms = 5000;
	uint8_t payload[airlink::kLinkTestControlPayloadSize] {};
	CHECK(airlink::encode_link_test_control(control, payload));
	airlink::LinkTestControl decoded{};
	CHECK(airlink::decode_link_test_control(payload, sizeof(payload), decoded));
	CHECK(decoded.payload_len == 256);
	CHECK(decoded.rate_hz == 500);
	CHECK(decoded.duration_ms == 5000);

	for (uint16_t payload_len : {static_cast<uint16_t>(31), static_cast<uint16_t>(513)}) {
		control.payload_len = payload_len;
		CHECK(!airlink::encode_link_test_control(control, payload));
	}

	control.payload_len = 256;

	for (uint16_t rate : {static_cast<uint16_t>(0), static_cast<uint16_t>(501)}) {
		control.rate_hz = rate;
		CHECK(!airlink::encode_link_test_control(control, payload));
	}

	control.rate_hz = 500;

	for (uint16_t duration : {static_cast<uint16_t>(999), static_cast<uint16_t>(10001)}) {
		control.duration_ms = duration;
		CHECK(!airlink::encode_link_test_control(control, payload));
	}
}

} // namespace

int main()
{
	test_crc_and_primitives();
	test_sensor_golden();
	test_sensor_boundaries_and_optional_sources();
	test_yaw_reference_round_trip();
	test_quaternion_rules();
	test_actuator_encoding();
	test_fixed_message_contracts();
	test_stream_parser();
	test_link_test_contracts();

	if (g_failures != 0) {
		std::fprintf(stderr, "%d AIRLINK protocol check(s) failed\n", g_failures);
		return EXIT_FAILURE;
	}

	std::puts("AIRLINK protocol checks passed");
	return EXIT_SUCCESS;
}
