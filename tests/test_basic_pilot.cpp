#include <cassert>
#include <cmath>
#include <type_traits>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

int main() {
    BasicPilotGains<> gains;
    BasicPilotInput<> input;
    BasicPilotGains<double> gains_d;
    (void)gains_d;
    static_assert(std::is_same<decltype(gains.P), float>::value, "default Real must be float");

    gains.P = 0.003f;
    gains.D = 0.1f;
    gains.DD = 0.05f;
    gains.PR = 0.01f;
    gains.FF = 0.2f;

    input.heading_error_deg = -9.0f;
    input.headingrate_deg_s = 0.5f;
    input.headingraterate_deg_s2 = -0.25f;
    input.heading_command_rate_deg_s = 0.1f;

    BasicPilotOutput<> out = compute_basic_pilot(input, gains, 10.0f);
    assert(std::fabs(out.Pgain - (-0.027f)) < 0.0001f);
    assert(std::fabs(out.Dgain - 0.05f) < 0.0001f);
    assert(std::fabs(out.DDgain - (-0.0125f)) < 0.0001f);
    assert(std::fabs(out.PRgain - (-0.03f)) < 0.0001f);
    assert(std::fabs(out.FFgain - 0.02f) < 0.0001f);
    assert(std::fabs(out.command_norm - 0.0005f) < 0.00011f);

    input.heading_error_deg = 1000.0f;
    gains.P = 1.0f;
    out = compute_basic_pilot(input, gains, 1.0f);
    assert(out.command_norm == 1.0f);

    return 0;
}
