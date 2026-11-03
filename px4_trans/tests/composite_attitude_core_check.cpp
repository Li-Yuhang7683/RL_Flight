#include "CompositeMath.hpp"

#include <cmath>
#include <cstdio>
#include <cstdint>

namespace
{

int g_failures = 0;

#define CHECK(condition) do { \
	if (!(condition)) { \
		std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
		++g_failures; \
	} \
} while (0)

constexpr float kDegToRad{composite::kPi / 180.f};

void test_quaternion_conventions()
{
	const composite::Quat identity{};
	CHECK(identity.normalized());
	const composite::Quat opposite{-identity.w, -identity.x, -identity.y, -identity.z};
	CHECK(composite::angleBetween(identity, opposite) < 1e-5f);

	const composite::Quat yaw_90 = composite::fromYaw(90.f * kDegToRad);
	const composite::Vec3 x_rotated = composite::rotate(yaw_90, {1.f, 0.f, 0.f});
	CHECK(std::fabs(x_rotated.x) < 1e-5f);
	CHECK(std::fabs(x_rotated.y - 1.f) < 1e-5f);
	CHECK(std::fabs(composite::yaw(yaw_90) - 90.f * kDegToRad) < 1e-5f);

	const composite::Quat yaw_180 = composite::fromYaw(composite::kPi);
	CHECK(std::fabs(composite::angleBetween(identity, yaw_180) - composite::kPi) < 1e-5f);
}

void test_assembly_transform()
{
	// C forward is 30 deg right of A and 30 deg left of B.
	// Calibration angles map vector components X->C, not C axes into X.
	const composite::Quat q_na = composite::fromYaw(-30.f * kDegToRad);
	const composite::Quat q_nb = composite::fromYaw(30.f * kDegToRad);
	const composite::Quat q_ca = composite::vehicleToCompositeQuatFromMountYaw(
					     -30.f * kDegToRad);
	const composite::Quat q_cb = composite::vehicleToCompositeQuatFromMountYaw(
					     30.f * kDegToRad);
	const composite::Quat q_ab = composite::inverse_unit(q_ca) * q_cb;
	CHECK(std::fabs(composite::yaw(q_ab) - 60.f * kDegToRad) < 1e-5f);
	CHECK(composite::angleBetween(q_ab, composite::assemblyQuatFromLocalAttitudes(
					       q_na, q_nb)) < 1e-5f);
	CHECK(composite::angleBetween(q_nb, q_na * q_ab) < 1e-5f);
	const composite::Quat q_a_composite = composite::attitudeInCompositeFrame(
					      q_na, -30.f * kDegToRad);
	const composite::Quat q_b_composite = composite::attitudeInCompositeFrame(
					      q_nb, 30.f * kDegToRad);
	CHECK(composite::angleBetween(q_a_composite, q_b_composite) < 1e-5f);
	CHECK(composite::angleBetween(q_a_composite, composite::Quat{}) < 1e-5f);

	// Independent geometric check: express C's forward axis in each aircraft.
	const composite::Vec3 c_forward_a = composite::rotate(composite::inverse_unit(q_ca), {1.f, 0.f, 0.f});
	const composite::Vec3 c_forward_b = composite::rotate(composite::inverse_unit(q_cb), {1.f, 0.f, 0.f});
	CHECK(std::fabs(std::atan2(c_forward_a.y, c_forward_a.x) - 30.f * kDegToRad) < 1e-5f);
	CHECK(std::fabs(std::atan2(c_forward_b.y, c_forward_b.x) + 30.f * kDegToRad) < 1e-5f);
	CHECK(composite::angleBetween(q_ab, composite::assemblyQuatFromMountYaw(-30.f * kDegToRad, 30.f * kDegToRad)) < 1e-5f);
	const composite::Vec3 master_axis = composite::rotate(q_ca, {1.f, 0.f, 0.f});
	const composite::Vec3 slave_axis = composite::rotate(q_cb, {1.f, 0.f, 0.f});
	CHECK(std::fabs(master_axis.x - std::sqrt(3.f) * 0.5f) < 1e-5f);
	CHECK(std::fabs(master_axis.y + 0.5f) < 1e-5f);
	CHECK(std::fabs(slave_axis.x - std::sqrt(3.f) * 0.5f) < 1e-5f);
	CHECK(std::fabs(slave_axis.y - 0.5f) < 1e-5f);

	// Check each branch independently so a future mapping bug cannot be hidden
	// by the symmetric 0.5/0.5 average.
	const composite::Vec3 force_c{-9.80665f, 0.f, 0.f};
	const composite::Vec3 force_a = composite::rotate(composite::inverse_unit(q_ca),
				      force_c);
	const composite::Vec3 force_b = composite::rotate(composite::inverse_unit(q_cb),
				      force_c);
	const composite::Vec3 mapped_a = composite::rotate(q_ca, force_a);
	const composite::Vec3 mapped_b = composite::rotate(q_cb, force_b);
	CHECK(std::fabs(mapped_a.x - force_c.x) < 1e-4f);
	CHECK(std::fabs(mapped_a.y) < 1e-4f);
	CHECK(std::fabs(mapped_b.x - force_c.x) < 1e-4f);
	CHECK(std::fabs(mapped_b.y) < 1e-4f);

	const composite::Quat level{};
	const composite::Quat yaw_only = composite::fromYaw(120.f * kDegToRad);
	CHECK(std::fabs(composite::tiltDifferenceBetween(level, yaw_only)) < 1e-5f);
	CHECK(std::fabs(composite::angleBetween(level, yaw_only) - 120.f * kDegToRad) < 1e-5f);

	const composite::Quat tilted = composite::fromEuler(3.f * kDegToRad, 0.f,
				     120.f * kDegToRad);
	CHECK(composite::tiltDifferenceBetween(level, tilted) > 2.9f * kDegToRad);
}

void test_zero_rate_average_and_lever_inputs()
{
	const composite::Quat q_ab = composite::fromYaw(60.f * kDegToRad);
	const composite::Vec3 omega_a{};
	const composite::Vec3 omega_b{};
	const composite::Vec3 omega_c = (omega_a + composite::rotate(q_ab, omega_b)) * 0.5f;
	CHECK(omega_c.norm() < 1e-6f);

	const composite::Vec3 force_a{0.f, 0.f, -9.80665f};
	const composite::Vec3 force_b{0.f, 0.f, -9.80665f};
	const composite::Vec3 force_c = (force_a + composite::rotate(q_ab, force_b)) * 0.5f;
	CHECK(std::fabs(force_c.z + 9.80665f) < 1e-4f);

	const composite::Vec3 r_a{0.2f, 0.f, 0.f};
	const composite::Vec3 r_b{-0.2f, 0.f, 0.f};
	const composite::Vec3 alpha{0.f, 2.f, 0.f};
	const composite::Vec3 term_a = composite::cross(alpha, r_a);
	const composite::Vec3 term_b = composite::cross(alpha, r_b);
	CHECK(std::fabs((term_a + term_b).norm()) < 1e-6f);
}

void test_clock_window_and_wrap()
{
	composite::ClockMapper mapper{};
	mapper.reset(1'000'000);
	uint32_t raw = 0xFFFFF000u;
	uint64_t arrival = 1'000'000;
	uint64_t mapped = 0;
	uint64_t age = 0;

	for (size_t i = 0; i < 500; ++i) {
		raw += 2'000;
		arrival += 2'000;
		mapper.add(raw, arrival, 500, mapped, age);
	}

	CHECK(mapper.offsetReady());
	CHECK(mapper.sampleCount() >= composite::ClockMapper::kMinSamples);
	CHECK(mapped != 0);
	CHECK(mapped <= arrival);
	CHECK(age <= composite::ClockMapper::kWindowUs);

	CHECK(!mapper.add(raw, arrival + 2'000, 500, mapped, age));
	CHECK(!mapper.add(raw - 10'000u, arrival + 4'000, 500, mapped, age));
}

void test_yaw_wrap_and_circular_mean()
{
	CHECK(std::fabs(composite::wrapPi(composite::kPi + 0.1f)
			- (-composite::kPi + 0.1f)) < 1e-5f);
	const float mean = composite::circularMeanYaw(composite::kPi - 0.1f,
			   -composite::kPi + 0.1f);
	CHECK(std::fabs(std::fabs(mean) - composite::kPi) < 1e-4f);

	composite::YawAnchor anchor{};
	anchor.reset(composite::kPi - 0.02f);
	CHECK(anchor.update(-composite::kPi + 0.02f, 0.1f, 1.f));
	CHECK(anchor.yaw() > composite::kPi - 0.1f);
}

} // namespace

int main()
{
	test_quaternion_conventions();
	test_assembly_transform();
	test_zero_rate_average_and_lever_inputs();
	test_clock_window_and_wrap();
	test_yaw_wrap_and_circular_mean();

	if (g_failures != 0) {
		std::fprintf(stderr, "composite core checks failed: %d\n", g_failures);
		return 1;
	}

	std::puts("composite core checks passed");
	return 0;
}
