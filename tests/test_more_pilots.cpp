#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

int main() {
    AbsolutePilotInput<> absolute_in;
    AbsolutePilotGains<> absolute_gains;
    absolute_in.heading_error_deg = 10.0f;
    absolute_in.heading_error_int_deg = 2.0f;
    absolute_in.headingrate_deg_s = 3.0f;
    absolute_in.heading_command_rate_deg_s = 4.0f;
    absolute_gains.P = 0.1f;
    absolute_gains.I = 0.2f;
    absolute_gains.D = 0.3f;
    absolute_gains.FF = 0.4f;
    AbsolutePilotOutput<> absolute_out = compute_absolute_pilot(absolute_in, absolute_gains);
    assert(std::fabs(absolute_out.command_norm - 3.9f) < 0.0001f);

    SimplePilotInput<> simple_in;
    SimplePilotGains<> simple_gains;
    simple_in.heading_error_deg = 10.0f;
    simple_in.heading_error_int_deg = 2.0f;
    simple_in.headingrate_deg_s = 3.0f;
    simple_gains.P = 0.1f;
    simple_gains.I = 0.2f;
    simple_gains.D = 0.3f;
    SimplePilotOutput<> simple_out = compute_simple_pilot(simple_in, simple_gains);
    assert(std::fabs(simple_out.command_norm - 2.3f) < 0.0001f);

    RatePilotInput<> rate_in;
    RatePilotGains<> rate_gains;
    rate_in.heading_error_deg = 20.0f;
    rate_in.headingrate_deg_s = 1.0f;
    rate_in.headingraterate_deg_s2 = 0.5f;
    rate_in.heading_command_rate_deg_s = 0.25f;
    rate_gains.D = 0.075f;
    rate_gains.DD = 0.075f;
    rate_gains.FF = 0.6f;
    rate_gains.max_turn_rate_deg_s = 2.0f;
    rate_gains.turn_rate_rate_deg_s2 = 0.5f;
    RatePilotOutput<> rate_out = compute_rate_pilot(rate_in, rate_gains);
    float rate_expected = (1.0f - 2.0f) * 0.075f + 0.5f * 0.075f + 0.25f * 0.6f;
    assert(std::fabs(rate_out.command_norm - rate_expected) < 0.0001f);

    GpsPilotInput<> gps_in;
    GpsPilotGains<> gps_gains;
    gps_in.heading_error_deg = 10.0f;
    gps_in.headingrate_deg_s = 1.0f;
    gps_in.headingraterate_deg_s2 = 0.5f;
    gps_in.heading_command_rate_deg_s = 0.25f;
    gps_gains.P = 0.003f;
    gps_gains.D = 0.1f;
    gps_gains.DD = 0.05f;
    gps_gains.FF = 0.6f;
    GpsPilotOutput<> gps_out = compute_gps_pilot(gps_in, gps_gains);
    assert(std::fabs(gps_out.command_norm - 0.305f) < 0.0001f);

    VmgPilotInput<> vmg_in;
    VmgPilotGains<> vmg_gains;
    vmg_in.heading_error_deg = 10.0f;
    vmg_in.headingrate_deg_s = 1.0f;
    vmg_in.headingraterate_deg_s2 = 0.5f;
    vmg_gains.P = 0.003f;
    vmg_gains.D = 0.09f;
    vmg_gains.DD = 0.075f;
    VmgPilotOutput<> vmg_out = compute_vmg_pilot(vmg_in, vmg_gains);
    assert(std::fabs(vmg_out.command_norm - 0.1575f) < 0.0001f);

    return 0;
}
