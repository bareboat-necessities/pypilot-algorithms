#pragma once

#include "control.hpp"

namespace pypilot_algorithms {

template<typename Real = float>
struct AbsolutePilotGains {
    Real P = Real(0);
    Real I = Real(0);
    Real D = Real(0);
    Real FF = Real(0);
};

template<typename Real = float>
struct AbsolutePilotInput {
    Real heading_error_deg = Real(0);
    Real heading_error_int_deg = Real(0);
    Real headingrate_deg_s = Real(0);
    Real heading_command_rate_deg_s = Real(0);
};

template<typename Real = float>
struct AbsolutePilotOutput {
    Real command_norm = Real(0);
    Real Pgain = Real(0);
    Real Igain = Real(0);
    Real Dgain = Real(0);
    Real FFgain = Real(0);
};

template<typename Real = float>
inline AbsolutePilotOutput<Real> compute_absolute_pilot(const AbsolutePilotInput<Real>& in,
                                                        const AbsolutePilotGains<Real>& gains,
                                                        CommandClamp clamp_mode = CommandClamp::raw,
                                                        Real command_limit = Real(1)) {
    AbsolutePilotOutput<Real> out;
    out.Pgain = gains.P * in.heading_error_deg;
    out.Igain = gains.I * in.heading_error_int_deg;
    out.Dgain = gains.D * in.headingrate_deg_s;
    out.FFgain = gains.FF * in.heading_command_rate_deg_s;
    Real command = out.Pgain + out.Igain + out.Dgain + out.FFgain;
    out.command_norm = apply_command_clamp(command, clamp_mode, command_limit);
    return out;
}

} // namespace pypilot_algorithms
