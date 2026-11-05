// Exercise the production selected-IMU reader using deterministic uORB stand-ins.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <initializer_list>

struct sensor_selection_s { uint64_t timestamp{}; uint32_t accel_device_id{}, gyro_device_id{}; };
struct vehicle_imu_s {
 uint64_t timestamp{}, timestamp_sample{};
 uint32_t accel_device_id{}, gyro_device_id{};
 float delta_angle[3]{}, delta_velocity[3]{};
 uint32_t delta_angle_dt{}, delta_velocity_dt{};
};
enum { sensor_selection, vehicle_imu };
#define ORB_ID(topic) topic
static sensor_selection_s selection;
static vehicle_imu_s imus[4];
static bool present[4];
static uint32_t generations[4], selection_generation;
namespace uORB {
class Subscription {
public:
 explicit Subscription(int topic, unsigned instance = 0) : _topic(topic), _instance(instance) {}
 bool ChangeInstance(unsigned instance) { _instance = instance; _generation = 0; return true; }
 bool copy(vehicle_imu_s *value) {
  if (!present[_instance]) { return false; }
  *value = imus[_instance]; _generation = generations[_instance]; return true;
 }
 bool update(vehicle_imu_s *value) {
  assert(_topic == vehicle_imu);
  return _generation != generations[_instance] && copy(value);
 }
 bool update(sensor_selection_s *value) {
  assert(_topic == sensor_selection);
  if (_generation == selection_generation) { return false; }
  *value = selection; _generation = selection_generation; return true;
 }
private:
 int _topic; unsigned _instance; uint32_t _generation{0};
};
}
#include "AirlinkImu_under_test.hpp"
#include "AirlinkProtocol.hpp"

static void publish(unsigned instance, uint32_t accel_id, uint32_t gyro_id, uint64_t timestamp)
{
 present[instance] = true;
 imus[instance] = {};
 auto &imu = imus[instance];
 imu.timestamp_sample = timestamp;
 imu.accel_device_id = accel_id;
 imu.gyro_device_id = gyro_id;
 imu.delta_angle_dt = 2000;
 imu.delta_velocity_dt = 4000;
 const float accel[3] {1.25f, -2.5f, -9.75f};
 const float gyro[3] {.125f, .25f, -.5f};
 for (unsigned i = 0; i < 3; ++i) {
  imu.delta_velocity[i] = accel[i] * .004f;
  imu.delta_angle[i] = gyro[i] * .002f;
 }
 ++generations[instance];
}
static void select(uint32_t accel_id, uint32_t gyro_id)
{
 selection = {1000000, accel_id, gyro_id}; ++selection_generation;
}
int main()
{
 airlink::ImuSource source;
 float xyz[3]{};
 // Instance 0 exists, but no selected IMU yet: do not silently send its data.
 publish(0, 10, 11, 1000000);
 source.update();
 assert(!source.acceleration(xyz) && !source.angularVelocity(xyz));
 select(20, 21);
 source.update();
 assert(source.timestampSample() == 0 && !source.acceleration(xyz));
 // Delayed advertisement of the selected, nonzero instance must be retried.
 publish(2, 20, 21, 1002000);
 source.update();
 assert(source.timestampSample() == 1002000);
 airlink::SensorInput input{};
 input.accel_valid = source.acceleration(input.accelerometer);
 input.gyro_valid = source.angularVelocity(input.gyro);
 input.imu_sample_time_us = static_cast<uint32_t>(source.timestampSample());
 assert(input.accel_valid && input.gyro_valid);
 assert(fabsf(input.accelerometer[0] - 1.25f) < 1e-6f);
 assert(fabsf(input.accelerometer[2] + 9.75f) < 1e-6f);
 assert(fabsf(input.gyro[1] - .25f) < 1e-6f);
 // Existing wire quantization is the only change to the calibrated SI rates.
 airlink::SensorState wire{};
 airlink::encode_sensor_state(input, wire);
 assert(wire.imu_sample_time_us == 1002000);
 assert(wire.accelerometer[0] == 160 && wire.accelerometer[1] == -320 && wire.accelerometer[2] == -1248);
 assert(wire.gyro[0] == 100 && wire.gyro[1] == 200 && wire.gyro[2] == -400);
 // No new message: retain its real sample time, rather than refreshing stale data.
 source.update(); assert(source.timestampSample() == 1002000);
 // A selection change must clear the old sample until the matching IMU appears.
 select(30, 31); source.update();
 assert(source.timestampSample() == 0 && !source.angularVelocity(xyz));
 publish(1, 30, 31, 1004000); source.update();
 assert(source.timestampSample() == 1004000);
 // Independent integration periods and invalid fields must not cause divide-by-zero.
 imus[1].delta_velocity_dt = 0; ++generations[1]; source.update();
 assert(!source.acceleration(xyz) && source.angularVelocity(xyz));
 imus[1].delta_angle[0] = std::numeric_limits<float>::quiet_NaN();
 ++generations[1]; source.update();
 assert(!source.angularVelocity(xyz));
 publish(1, 30, 31, 0); source.update();
 assert(!source.acceleration(xyz) && !source.angularVelocity(xyz));
 // An instance whose device IDs change must not emit data for another sensor.
 publish(1, 30, 31, 1006000); source.update();
 publish(1, 40, 41, 1008000); source.update();
 assert(source.timestampSample() == 0 && !source.acceleration(xyz));
 publish(3, 30, 31, 1010000); source.update();
 assert(source.timestampSample() == 1010000 && source.acceleration(xyz));
 // Reproduce independent PX4 voting: accel from instance 0, gyro from 3.
 imus[0].delta_velocity[0] = .008f; ++generations[0];
 imus[3].delta_angle[2] = -.0015f; ++generations[3];
 select(10, 31); source.update();
 assert(source.acceleration(xyz) && fabsf(xyz[0] - 2.f) < 1e-6f);
 assert(source.angularVelocity(xyz) && fabsf(xyz[2] + .75f) < 1e-6f);
 assert(source.gyroInstance() == 3);
 assert(source.accelerationTimestamp() == 1000000 && source.timestampSample() == 1010000);
 // Exact reported failure: flags=60 and zero IMU timestamp must never get half weight.
 assert(!airlink::sensor_imu_valid(60, 0));
 const uint16_t required = airlink::AccelValid | airlink::GyroValid | airlink::SampleFresh;
 assert(airlink::sensor_imu_valid(required, 1010000));
 assert(!airlink::sensor_imu_valid(required, 0));
 for (uint16_t bit : {airlink::AccelValid, airlink::GyroValid, airlink::SampleFresh}) {
  assert(!airlink::sensor_imu_valid(required & ~bit, 1010000));
 }
 puts("Airlink calibrated selected IMU checks passed");
}
