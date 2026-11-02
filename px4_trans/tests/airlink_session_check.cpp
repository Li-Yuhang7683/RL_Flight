#include "AirlinkSession.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>

namespace
{

int g_failures = 0;

#define CHECK(condition) do { \
	if (!(condition)) { \
		std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
		++g_failures; \
	} \
} while (0)

airlink::LocalStatus ready_local(bool master)
{
	airlink::LocalStatus local{};
	local.local_ready = true;
	local.local_rc_valid = true;
	local.safety_gate_allow = true;
	local.pair_mode_selected = true;
	(void)master;
	return local;
}

void pump(airlink::Session &from, airlink::Session &to, uint64_t now_us)
{
	airlink::Frame frame{};

	while (from.pop_tx(frame)) {
		to.receive(frame, now_us);
	}
}

void exchange_heartbeat(airlink::Session &master, airlink::Session &slave, uint64_t now_us)
{
	(void)master.tick(now_us);
	(void)slave.tick(now_us);
	pump(master, slave, now_us);
	pump(slave, master, now_us);
}

void establish_ready(airlink::Session &master, airlink::Session &slave, uint64_t start_us)
{
	exchange_heartbeat(master, slave, start_us);
	pump(master, slave, start_us + 1);
	pump(slave, master, start_us + 2);
	pump(master, slave, start_us + 3);
	pump(slave, master, start_us + 4);

	CHECK(master.state() == airlink::LinkState::Ready);
	CHECK(slave.state() == airlink::LinkState::Ready);
	exchange_heartbeat(master, slave, start_us + 2'000);
}

void establish_pair_active(airlink::Session &master, airlink::Session &slave,
			   uint64_t start_us)
{
	establish_ready(master, slave, start_us);
	CHECK(master.request_control_enable(start_us + 20'000));
	pump(master, slave, start_us + 20'001);
	pump(slave, master, start_us + 20'002);
	CHECK(master.state() == airlink::LinkState::PairControlReady);
	CHECK(slave.state() == airlink::LinkState::PairControlReady);
	CHECK(master.request_pair_arm(start_us + 30'000));
	airlink::Frame arm_duplicate{};
	CHECK(master.pop_tx(arm_duplicate));
	slave.receive(arm_duplicate, start_us + 30'001);
	CHECK(slave.state() == airlink::LinkState::PairArmPending);
	CHECK(slave.notify_local_arm_result(true, start_us + 30'002));
	pump(slave, master, start_us + 30'003);
	CHECK(master.state() == airlink::LinkState::PairArmPending);
	CHECK(master.notify_local_arm_result(true, start_us + 30'004));
	CHECK(master.state() == airlink::LinkState::PairArmedGround);

	airlink::ActuatorCommand command{};
	command.upper_motor = command.lower_motor = 0.f;
	command.servos[0] = command.servos[1] = command.servos[2] = 0.f;
	CHECK(master.queue_actuator_command(command, 0.f, start_us + 30'005));
	pump(master, slave, start_us + 30'006);
	CHECK(slave.state() == airlink::LinkState::PairActive);

	airlink::SensorState sensor{};
	sensor.status_flags = airlink::AccelValid | airlink::GyroValid | airlink::AttitudeValid
			      | airlink::AngularVelocityValid | airlink::SampleFresh
			      | airlink::LocalRcValid | airlink::SafetyGateAllow
			      | airlink::AirlinkControlActive;
	sensor.attitude_q[0] = 32767;
	CHECK(slave.queue_sensor_state(sensor, start_us + 30'007));
	pump(slave, master, start_us + 30'008);
	CHECK(master.state() == airlink::LinkState::PairActive);

	// A duplicate PAIR_ARM must return the cached result and never ask the
	// wrapper to arm a second time after this epoch is already active.
	airlink::SessionActions duplicate_actions =
		slave.receive(arm_duplicate, start_us + 30'009);
	CHECK(!duplicate_actions.request_arm);
	CHECK(slave.state() == airlink::LinkState::PairActive);
}

void test_handshake_and_pair_lifecycle()
{
	constexpr uint32_t config_crc = 0x12345678u;
	auto master = std::make_unique<airlink::Session>(config_crc);
	auto slave = std::make_unique<airlink::Session>(config_crc);
	master->start(airlink::Role::Master, 0x11111111u, ready_local(true), 1'000);
	slave->start(airlink::Role::Slave, 0x22222222u, ready_local(false), 1'000);
	establish_pair_active(*master, *slave, 2'000);
	CHECK(master->peer_session_id() == slave->session_id());
	CHECK(slave->peer_session_id() == master->session_id());

	const airlink::SessionActions master_lost = master->tick(50'000);
	const airlink::SessionActions slave_lost = slave->tick(50'000);
	CHECK(master->state() == airlink::LinkState::PairLost);
	CHECK(slave->state() == airlink::LinkState::PairLost);
	CHECK(master_lost.pair_lost);
	CHECK(slave_lost.pair_lost);
	CHECK(master_lost.request_disarm);
	CHECK(slave_lost.request_disarm);
	CHECK(master->pair_reattach_latched());
	CHECK(slave->pair_reattach_latched());

	master->set_local_status(ready_local(true));
	slave->set_local_status(ready_local(false));
	CHECK(!master->request_control_enable(50'001));
}

void test_handshake_config_mismatch()
{
	auto master = std::make_unique<airlink::Session>(0x11111111u);
	auto slave = std::make_unique<airlink::Session>(0x22222222u);
	master->start(airlink::Role::Master, 10, ready_local(true), 10'000);
	slave->start(airlink::Role::Slave, 20, ready_local(false), 10'000);
	exchange_heartbeat(*master, *slave, 11'000);
	pump(*master, *slave, 11'001);
	pump(*slave, *master, 11'002);
	CHECK(master->state() == airlink::LinkState::Discovered);
	CHECK(slave->state() == airlink::LinkState::HandshakePending);
	CHECK(!master->request_control_enable(11'003));
}

void test_same_role_never_links()
{
	auto first = std::make_unique<airlink::Session>(1);
	auto second = std::make_unique<airlink::Session>(1);
	first->start(airlink::Role::Master, 10, ready_local(true), 20'000);
	second->start(airlink::Role::Master, 20, ready_local(true), 20'000);
	exchange_heartbeat(*first, *second, 21'000);
	CHECK(first->state() == airlink::LinkState::Down);
	CHECK(second->state() == airlink::LinkState::Down);
	CHECK(first->peer_session_id() == 0);
	CHECK(second->peer_session_id() == 0);
}

void test_unknown_command_gets_unsupported_ack()
{
	auto master = std::make_unique<airlink::Session>(1);
	auto slave = std::make_unique<airlink::Session>(1);
	master->start(airlink::Role::Master, 10, ready_local(true), 25'000);
	slave->start(airlink::Role::Slave, 20, ready_local(false), 25'000);
	establish_ready(*master, *slave, 26'000);

	uint8_t payload[8] {0, 0, 0, 0, 0x44, 0x33, 0x22, 0x11};
	uint8_t encoded[airlink::kMaxFrameSize] {};
	size_t frame_len = 0;
	airlink::Header header{};
	header.frame_type = airlink::FrameType::Command;
	header.message_id = 0x20;
	header.flags = airlink::AckRequired;
	header.source_role = airlink::Role::Master;
	header.dest_role = airlink::Role::Slave;
	header.source_session = master->session_id();
	header.sequence = 123;
	header.payload_len = sizeof(payload);
	CHECK(airlink::build_frame(header, payload, sizeof(payload), encoded, sizeof(encoded),
				   frame_len));
	airlink::Frame frame{};
	airlink::StreamParser parser;
	parser.feed(encoded, frame_len, 28'000);
	CHECK(parser.next(frame));
	airlink::Ack expected_ack{};
	expected_ack.target_peer_session = master->session_id();
	expected_ack.event_id = 0x11223344u;
	expected_ack.acked_message_id = 0x20;
	expected_ack.result = airlink::AckResult::Unsupported;
	uint8_t ack_payload[airlink::kAckPayloadSize] {};
	CHECK(airlink::encode_ack(expected_ack, ack_payload));
	airlink::Header ack_header{};
	ack_header.frame_type = airlink::FrameType::Command;
	ack_header.flags = airlink::Response;
	ack_header.message_id = static_cast<uint8_t>(airlink::MessageId::Ack);
	ack_header.source_role = airlink::Role::Slave;
	ack_header.dest_role = airlink::Role::Master;
	ack_header.source_session = slave->session_id();
	ack_header.sequence = 1;
	ack_header.payload_len = sizeof(ack_payload);
	uint8_t ack_encoded[airlink::kMaxFrameSize] {};
	size_t ack_len = 0;
	CHECK(airlink::build_frame(ack_header, ack_payload, sizeof(ack_payload), ack_encoded,
				   sizeof(ack_encoded), ack_len));
	(void)slave->receive(frame, 28'000);
	CHECK(slave->unknown_messages() == 1);
	CHECK(slave->tx_queue_size() == 1);
	airlink::Frame ack_frame{};
	CHECK(slave->pop_tx(ack_frame));
	CHECK(ack_frame.header.message_id == static_cast<uint8_t>(airlink::MessageId::Ack));
	airlink::Ack ack{};
	CHECK(airlink::decode_ack(ack_frame.payload, ack_frame.header.payload_len, ack));
	CHECK(ack.result == airlink::AckResult::Unsupported);
	CHECK(ack.acked_message_id == 0x20);
	CHECK(ack.event_id == 0x11223344u);
}

void test_reliable_retry_limit()
{
	auto master = std::make_unique<airlink::Session>(1);
	auto slave = std::make_unique<airlink::Session>(1);
	master->start(airlink::Role::Master, 10, ready_local(true), 30'000);
	slave->start(airlink::Role::Slave, 20, ready_local(false), 30'000);
	exchange_heartbeat(*master, *slave, 31'000);
	uint64_t now = 31'001;
	int sends = 0;

	for (int iteration = 0; iteration < 6; ++iteration) {
		airlink::Frame frame{};

		while (master->pop_tx(frame)) {
			if (frame.header.message_id == static_cast<uint8_t>(airlink::MessageId::LinkHandshake)) {
				++sends;

				if (sends > 1) {
					CHECK((frame.header.flags & airlink::Retry) != 0);
				}
			}
		}

		now += 5'000;
		(void)master->tick(now);
	}

	CHECK(sends >= 4);
	CHECK(sends <= 5);
	CHECK(master->state() == airlink::LinkState::Discovered);
}

void test_link_test_round_trip()
{
	auto master = std::make_unique<airlink::Session>(1);
	auto slave = std::make_unique<airlink::Session>(1);
	master->start(airlink::Role::Master, 10, ready_local(true), 40'000);
	slave->start(airlink::Role::Slave, 20, ready_local(false), 40'000);
	establish_ready(*master, *slave, 41'000);
	airlink::LinkTestRequest request{};
	request.payload_len = 64;
	request.rate_hz = 500;
	request.duration_ms = 1000;
	CHECK(master->start_link_test(request, 61'000));
	pump(*master, *slave, 61'001);
	pump(*slave, *master, 61'002);
	CHECK(master->link_test_active());
	CHECK(slave->link_test_active());

	uint64_t now = 61'003;
	bool finished = false;

	for (int iteration = 0; iteration < 5000; ++iteration) {
		master->tick(now);
		pump(*master, *slave, now);
		slave->tick(now);
		pump(*slave, *master, now + 1);
		now += 2000;

		if (!master->link_test_active()) {
			finished = true;
			break;
		}
	}

	CHECK(finished);
	CHECK(master->link_test_stats().sent > 400);
	CHECK(master->link_test_stats().echoed > 400);
	CHECK(master->link_test_stats().missing == 0);
	CHECK(master->link_test_stats().offered_bytes_per_second > 0);
}

} // namespace

int main()
{
	test_handshake_and_pair_lifecycle();
	test_handshake_config_mismatch();
	test_same_role_never_links();
	test_unknown_command_gets_unsupported_ack();
	test_reliable_retry_limit();
	test_link_test_round_trip();

	if (g_failures != 0) {
		std::fprintf(stderr, "%d AIRLINK session check(s) failed\n", g_failures);
		return EXIT_FAILURE;
	}

	std::puts("AIRLINK session checks passed");
	return EXIT_SUCCESS;
}
