#pragma once

#include "control.hpp"

namespace pypilot_algorithms {

template<typename Real = float>
struct WindPilotGains {
    Real P = Real(0);
    Real D = Real(0);
    Real DD = Real(0);
    Real WG = Real(0);
};

template<typename Real = float>
struct WindPilotInput {
    Real wind_error_deg = Real(0);
    Real headingrate_deg_s = Real(0);
    Real headingraterate_deg_s2 = Real(0);
    Real wind_gust_kn = Real(0);
};

template<typename Real = float>
struct WindPilotOutput {
    Real command_norm = Real(0);
    Real Pgain = Real(0);
    Real Dgain = Real(0);
    Real DDgain = Real(0);
    Real WGgain = Real(0);
};

template<typename Real = float>
inline WindPilotOutput<Real> compute_wind_pilot(const WindPilotInput<Real>& in,
                                                 const WindPilotGains<Real>& gains,
                                                 Real command_limit = Real(1)) {
    WindPilotOutput<Real> out;
    out.Pgain = gains.P * in.wind_error_deg;
    out.Dgain = gains.D * in.headingrate_deg_s;
    out.DDgain = gains.DD * in.headingraterate_deg_s2;
    out.WGgain = gains.WG * in.wind_gust_kn;
    out.command_norm = clamp_command(out.Pgain + out.Dgain + out.DDgain + out.WGgain, command_limit);
    return out;
}

} // namespace pypilot_algorithms
