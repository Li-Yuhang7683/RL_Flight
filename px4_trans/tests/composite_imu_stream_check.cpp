// Compile the production MAVLink stream with small uORB/MAVLink stand-ins.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
using param_t = int;
static uint64_t clock_us = 1000000;
uint64_t hrt_absolute_time() { return clock_us; }
param_t param_find(const char *name) { return !strcmp(name, "AIRLINK_ROLE") ? 0 : !strcmp(name, "CEKF_MSTR_YAW") ? 1 : 2; }
static int32_t role = 1;
static float yaw_a = -30.f, yaw_b = 30.f;
int param_get(param_t p, void *v) { if (p == 0) { *static_cast<int32_t *>(v) = role; } else { *static_cast<float *>(v) = p == 1 ? yaw_a : yaw_b; } return 0; }
struct sensor_accel_s { uint64_t timestamp_sample{}; float x{}, y{}, z{}; };
struct sensor_gyro_s { uint64_t timestamp_sample{}; float x{}, y{}, z{}; };
struct airlink_sensor_state_s { uint64_t timestamp{}; uint16_t status_flags{}; int16_t accelerometer_raw[3]{}, gyro_raw[3]{}; };
struct composite_attitude_estimate_s {
 uint64_t timestamp{}; float q[4]{}, specific_force[3]{}, angular_velocity[3]{}, angular_acceleration[3]{};
 uint8_t reset_counter{}; uint16_t health_flags{}; uint32_t source_age_a_us{}, source_age_b_us{};
 float yaw_sigma_rad{}; bool valid{};
};
struct composite_estimator_status_s {
 uint64_t timestamp{}; bool enabled{}, attitude_initialized{}, peer_connected{};
 uint8_t state{}; float weight_a{}, weight_b{}; uint32_t frame_count{};
};
struct composite_yaw_reference_s {
 static constexpr uint8_t FLAG_VALID = 1;
 uint64_t timestamp{}; float yaw_rad{}; uint8_t flags{};
};
static composite_attitude_estimate_s estimate;
static composite_estimator_status_s status;
static composite_yaw_reference_s yaw;
static sensor_accel_s accel;
static sensor_gyro_s gyro;
static airlink_sensor_state_s peer;
enum { sensor_accel, sensor_gyro, airlink_sensor_state, composite_attitude_estimate, composite_estimator_status, composite_yaw_reference };
#define ORB_ID(x) x
namespace uORB {
struct Subscription {
 int id;
 bool advertised() { return true; }
 bool update(sensor_accel_s *v) { *v = accel; return true; }
 bool update(sensor_gyro_s *v) { *v = gyro; return true; }
 bool update(airlink_sensor_state_s *v) { *v = peer; return true; }
 bool update(composite_attitude_estimate_s *v) { *v = estimate; return true; }
 bool update(composite_estimator_status_s *v) { *v = status; return true; }
 bool update(composite_yaw_reference_s *v) { *v = yaw; return true; }
};
}
#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT 251
#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN 18
#define MAVLINK_NUM_NON_PAYLOAD_BYTES 12
struct mavlink_named_value_float_t { uint32_t time_boot_ms; char name[10]; float value; };
struct { float data[51]; } output;
static int packets = 0;
static const char *const expected_names[51] {
 "A_ACC_R_X", "A_ACC_R_Y", "A_ACC_R_Z", "B_ACC_R_X", "B_ACC_R_Y", "B_ACC_R_Z",
 "A_ACC_C_X", "A_ACC_C_Y", "A_ACC_C_Z", "B_ACC_C_X", "B_ACC_C_Y", "B_ACC_C_Z",
 "A_GYR_R_X", "A_GYR_R_Y", "A_GYR_R_Z", "B_GYR_R_X", "B_GYR_R_Y", "B_GYR_R_Z",
 "A_GYR_C_X", "A_GYR_C_Y", "A_GYR_C_Z", "B_GYR_C_X", "B_GYR_C_Y", "B_GYR_C_Z",
 "C_ROLL", "C_PITCH", "C_YAW", "C_ACC_X", "C_ACC_Y", "C_ACC_Z",
 "C_GYR_X", "C_GYR_Y", "C_GYR_Z", "C_ALP_X", "C_ALP_Y", "C_ALP_Z",
 "CE_RUN", "CE_EN", "CE_INIT", "CE_STATE", "CE_LINK", "CE_W_A", "CE_W_B", "CE_VALID",
 "CE_RESET", "CE_FLAGS", "CE_AGE_A", "CE_AGE_B", "CE_FRAMES", "CE_YAWREF", "CE_YSIG"
};
void mavlink_msg_named_value_float_send_struct(int, const mavlink_named_value_float_t *msg)
{
 unsigned channel = packets % 51;
 assert(msg->name[9] == '\0');
 assert(!strcmp(msg->name, expected_names[channel]));
 assert(msg->time_boot_ms == clock_us / 1000);
 output.data[channel] = msg->value;
 ++packets;
}
struct Mavlink { int get_channel() { return 0; } };
class MavlinkStream {
public:
 explicit MavlinkStream(Mavlink *m) : _mavlink(m) {}
 virtual ~MavlinkStream() = default;
 virtual const char *get_name() const = 0;
 virtual uint16_t get_id() = 0;
 virtual unsigned get_size() = 0;
 virtual bool send() = 0;
protected:
 Mavlink *_mavlink;
};
#include "COMPOSITE_IMU_under_test.hpp"
static bool near(float x, float y) { return fabsf(x-y) < 1e-5f; }
int main()
{
 Mavlink mavlink;
 auto *stream = MavlinkStreamCompositeImu::new_instance(&mavlink);
 accel = {clock_us, 1.f, 0.f, -9.f};
 gyro = {clock_us, 0.f, 1.f, 2.f};
 peer.timestamp = clock_us;
 peer.status_flags = 3;
 peer.accelerometer_raw[0] = 128;
 peer.accelerometer_raw[2] = -1152;
 peer.gyro_raw[1] = 800;
 peer.gyro_raw[2] = 1600;
 assert(stream->send());
 assert(packets == 51);
 assert(stream->get_size() == 51 * (18 + 12));
 assert(near(output.data[0], 1) && near(output.data[3], 1));
 assert(near(output.data[6], sqrtf(3.f)/2) && near(output.data[7], -.5f));
 assert(near(output.data[9], sqrtf(3.f)/2) && near(output.data[10], .5f));
 assert(near(output.data[18], .5f) && near(output.data[19], sqrtf(3.f)/2));
 assert(near(output.data[21], -.5f) && near(output.data[22], sqrtf(3.f)/2));
 assert(output.data[8] == -9 && output.data[11] == -9);
 assert(output.data[20] == 2 && output.data[23] == 2);
 // Missing peer validity must not turn invalid sensor values into zeros.
 peer.status_flags = 0;
 assert(stream->send());
 assert(std::isnan(output.data[3]) && std::isnan(output.data[15]));
 peer.status_flags = 3;
 clock_us += 200000;
 assert(stream->send());
 for (int i = 0; i < 24; ++i) { assert(std::isnan(output.data[i])); }
 assert(output.data[36] == 0 && output.data[43] == 0);
 // Live mounting parameter update, and stable A/B labels on the slave.
 role = 2;
 yaw_b = 0;
 accel.timestamp_sample = gyro.timestamp_sample = clock_us;
 assert(stream->send());
 assert(std::isnan(output.data[0]) && std::isnan(output.data[12]));
 assert(output.data[3] == 1 && output.data[9] == 1 && output.data[10] == 0);
 assert(output.data[16] == 1 && output.data[22] == 1);
 assert(packets == 4 * 51);
 role = 0;
 assert(stream->send());
 assert(packets == 5 * 51);
 // CEKF telemetry is independent of raw IMU role gating.
 status = {clock_us, true, true, true, 1, .5f, .5f, 1234};
 estimate.timestamp = clock_us;
 estimate.valid = true;
 estimate.q[0] = estimate.q[3] = sqrtf(.5f); // yaw +90 deg
 estimate.specific_force[0] = 1.25f;
 estimate.specific_force[2] = -9.8f;
 estimate.angular_velocity[1] = -.3f;
 estimate.angular_acceleration[2] = .75f;
 estimate.reset_counter = 7;
 estimate.health_flags = 44;
 estimate.source_age_b_us = 2500;
 estimate.yaw_sigma_rad = .1f;
 yaw = {clock_us, 1.f, 1};
 assert(stream->send());
 assert(output.data[36] == 1 && output.data[37] == 1 && output.data[38] == 1);
 assert(output.data[39] == 1 && output.data[40] == 1);
 assert(output.data[41] == .5f && output.data[42] == .5f && output.data[43] == 1);
 assert(near(output.data[24], 0) && near(output.data[25], 0) && near(output.data[26], 90));
 assert(near(output.data[27], 1.25f) && near(output.data[29], -9.8f));
 assert(near(output.data[31], -.3f) && near(output.data[35], .75f));
 assert(output.data[44] == 7 && output.data[45] == 44 && output.data[46] == 0);
 assert(near(output.data[47], 2.5f) && output.data[48] == 1234);
 assert(near(output.data[49], 57.2957795f) && near(output.data[50], .1f));
 // Link loss keeps fusion live, while switching weights and mode.
 status.peer_connected = false; status.state = 2; status.weight_a = 1; status.weight_b = 0;
 assert(stream->send());
 assert(output.data[40] == 0 && output.data[41] == 1 && output.data[42] == 0 && output.data[43] == 1);
 // Disabled but still running must not replay a fresh, cached estimate.
 status.enabled = false;
 assert(stream->send());
 assert(output.data[36] == 1 && output.data[37] == 0 && output.data[43] == 0);
 for (int i = 24; i < 36; ++i) { assert(std::isnan(output.data[i])); }
 assert(std::isnan(output.data[49]));
 status.enabled = true;
 // Stale estimate while module heartbeat is live: no frozen curves.
 clock_us += 200000;
 status.timestamp = clock_us;
 assert(stream->send());
 assert(output.data[36] == 1 && output.data[43] == 0 && std::isnan(output.data[26]));
 // Module stop: explicit RUN/VALID=0; all other channels unavailable.
 clock_us += 200000;
 assert(stream->send());
 for (unsigned i = 24; i < 51; ++i) {
  if (i == 36 || i == 43) { assert(output.data[i] == 0); }
  else { assert(std::isnan(output.data[i])); }
 }
 delete stream;
 puts("composite IMU stream checks passed");
}
