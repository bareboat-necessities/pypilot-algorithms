#include <cassert>
#include <cmath>
#include <type_traits>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

int main() {
    WindPilotGains<> gains;
    WindPilotInput<> input;
    WindPilotGains<double> gains_d;
    (void)gains_d;
    static_assert(std::is_same<decltype(gains.P), float>::value, "default Real must be float");

    gains.P = 0.003f;
    gains.D = 0.1f;
    gains.DD = 0.05f;
    gains.WG = -0.02f;

    input.wind_error_deg = -10.0f;
    input.headingrate_deg_s = 0.5f;
    input.headingraterate_deg_s2 = -0.2f;
    input.wind_gust_kn = 3.0f;

    WindPilotOutput<> out = compute_wind_pilot(input, gains, 10.0f);
    assert(std::fabs(out.Pgain - (-0.03f)) < 0.0001f);
    assert(std::fabs(out.Dgain - 0.05f) < 0.0001f);
    assert(std::fabs(out.DDgain - (-0.01f)) < 0.0001f);
    assert(std::fabs(out.WGgain - (-0.06f)) < 0.0001f);
    assert(std::fabs(out.command_norm - (-0.05f)) < 0.0001f);

    gains.P = 10.0f;
    out = compute_wind_pilot(input, gains, 1.0f);
    assert(out.command_norm == -1.0f);

    return 0;
}
