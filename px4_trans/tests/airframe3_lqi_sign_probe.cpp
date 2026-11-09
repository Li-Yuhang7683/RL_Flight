// Offline-only direction probe for the embedded airframe-3 LQI model.
// Reuses the exact C++ control core and compiled payload as PX4. It does NOT
// command actuators, verify physical aerodynamic signs, or authorize flight.
#include "../px4/src/modules/nn_control/LqiCoaxialCollectiveCore.hpp"
#include "../px4/src/modules/nn_control/LqiCoaxialCollectivePayload.hpp"

#include <cmath>
#include <cstdio>

namespace {
constexpr float kPi{3.14159265358979323846f};
constexpr float kG{9.80665f};

struct Case {
    const char *name;
    float roll_deg;
    float pitch_deg;
    float yaw_rate_rad_s;
};

bool run_case(const Case &sample)
{
    LqiCoaxialCollectiveCore::Input input{};
    const float roll = sample.roll_deg * kPi / 180.0f;
    const float pitch = sample.pitch_deg * kPi / 180.0f;
    const float cr = std::cos(roll * 0.5f);
    const float sr = std::sin(roll * 0.5f);
    const float cp = std::cos(pitch * 0.5f);
    const float sp = std::sin(pitch * 0.5f);

    // FRD body-to-NED Hamilton quaternion; zero yaw.
    input.attitude_q_wb[0] = cr * cp;
    input.attitude_q_wb[1] = sr * cp;
    input.attitude_q_wb[2] = cr * sp;
    input.attitude_q_wb[3] = -sr * sp;
    input.target_attitude_q_wb[0] = 1.0f; // target level
    input.angular_velocity_b[2] = sample.yaw_rate_rad_s;
    input.acceleration_b[2] = -kG;
    // Non-zero illustrative collective, not connected to RC or output driver.
    input.collective_base = 0.55f;

    float persistent[LqiCoaxialCollectiveCore::kPersistentSize]{};
    float next_persistent[LqiCoaxialCollectiveCore::kPersistentSize]{};
    LqiCoaxialCollectiveCore::ControlOutput output{};
    const bool ok = LqiCoaxialCollectiveCore::stepFromState(
        lqi_coaxial_collective::kModel,
        input,
        persistent,
        output,
        next_persistent
    );
    if (!ok) {
        std::printf("%-15s INVALID\n", sample.name);
        return false;
    }

    std::printf(
        "%-15s  upper=% .5f lower=% .5f  servo1=% .5f servo2=% .5f servo3=% .5f\n",
        sample.name,
        static_cast<double>(output.upper),
        static_cast<double>(output.lower),
        static_cast<double>(output.servos[0]),
        static_cast<double>(output.servos[1]),
        static_cast<double>(output.servos[2])
    );
    return true;
}
} // namespace

int main()
{
    std::printf("OFFLINE AIRFRAME-3 LQI DIRECTION PROBE (no actuator command)\n");
    std::printf("Model assumes FRD +X forward, +Y right, +Z down.\n");
    std::printf("Inputs are actual attitude errors relative to a level reference.\n");
    std::printf("Outputs are raw controller-domain normalized values.\n");
    std::printf("These do NOT prove physical vane thrust-moment signs.\n\n");
    const Case tests[]{
        {"level",        0.0f, 0.0f, 0.0f},
        {"roll_right",   8.0f, 0.0f, 0.0f},
        {"roll_left",   -8.0f, 0.0f, 0.0f},
        {"pitch_up",     0.0f, 8.0f, 0.0f},
        {"pitch_down",   0.0f,-8.0f, 0.0f},
        {"yaw_rate_pos", 0.0f, 0.0f, 0.3f},
        {"yaw_rate_neg", 0.0f, 0.0f,-0.3f},
    };
    bool all_good = true;
    for (const Case &sample : tests) {
        all_good = run_case(sample) && all_good;
    }
    return all_good ? 0 : 1;
}
