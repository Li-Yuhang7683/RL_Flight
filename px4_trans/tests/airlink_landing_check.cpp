#include "AirlinkLandingGuard.hpp"

#include <cstdio>
#include <cstdlib>

namespace
{

int failures = 0;

#define CHECK(condition) do { \
	if (!(condition)) { \
		std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
		++failures; \
	} \
} while (0)

void test_impact_confirmation_and_reset()
{
	airlink_landing::Guard guard;
	guard.start(1'000);
	CHECK(guard.update(1'000, true, false, 20, 150, 15'000)
	      == airlink_landing::Termination::None);
	CHECK(guard.update(1'019, true, false, 20, 150, 15'000)
	      == airlink_landing::Termination::None);
	CHECK(guard.update(1'020, true, false, 20, 150, 15'000)
	      == airlink_landing::Termination::Impact);

	guard.start(2'000);
	CHECK(guard.update(2'000, true, false, 20, 150, 15'000)
	      == airlink_landing::Termination::None);
	CHECK(guard.update(2'010, false, false, 20, 150, 15'000)
	      == airlink_landing::Termination::None);
	CHECK(guard.update(2'019, true, false, 20, 150, 15'000)
	      == airlink_landing::Termination::None);
	CHECK(guard.update(2'038, true, false, 20, 150, 15'000)
	      == airlink_landing::Termination::None);
	CHECK(guard.update(2'039, true, false, 20, 150, 15'000)
	      == airlink_landing::Termination::Impact);
}

void test_flip_confirmation()
{
	airlink_landing::Guard guard;
	guard.start(10'000);
	CHECK(guard.update(10'000, false, true, 20, 150, 15'000)
	      == airlink_landing::Termination::None);
	CHECK(guard.update(10'149, false, true, 20, 150, 15'000)
	      == airlink_landing::Termination::None);
	CHECK(guard.update(10'150, false, true, 20, 150, 15'000)
	      == airlink_landing::Termination::Flip);
}

void test_timeout_boundary()
{
	airlink_landing::Guard guard;
	guard.start(100);
	CHECK(guard.update(15'100, false, false, 20, 150, 15'000)
	      == airlink_landing::Termination::None);
	CHECK(guard.update(15'101, false, false, 20, 150, 15'000)
	      == airlink_landing::Termination::Timeout);
}

} // namespace

int main()
{
	test_impact_confirmation_and_reset();
	test_flip_confirmation();
	test_timeout_boundary();

	if (failures != 0) {
		std::fprintf(stderr, "%d AIRLINK landing check(s) failed\n", failures);
		return EXIT_FAILURE;
	}

	std::puts("AIRLINK landing checks passed");
	return EXIT_SUCCESS;
}
