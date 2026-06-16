#pragma once

#include "control.hpp"

namespace pypilot_algorithms {

template<typename Real = float>
struct SimplePilotGains {
    Real P = Real(0);
    Real I = Real(0);
    Real D = Real(0);
};

template<typename Real = float>
struct SimplePilotInput {
    Real heading_error_deg = Real(0);
    Real heading_error_int_deg = Real(0);
    Real headingrate_deg_s = Real(0);
};

template<typename Real = float>
struct SimplePilotOutput {
    Real command_norm = Real(0);
    Real Pgain = Real(0);
    Real Igain = Real(0);
    Real Dgain = Real(0);
};

template<typename Real = float>
inline SimplePilotOutput<Real> compute_simple_pilot(const SimplePilotInput<Real>& in,
                                                    const SimplePilotGains<Real>& gains,
                                                    CommandClamp clamp_mode = CommandClamp::raw,
                                                    Real command_limit = Real(1)) {
    SimplePilotOutput<Real> out;
    out.Pgain = gains.P * in.heading_error_deg;
    out.Igain = gains.I * in.heading_error_int_deg;
    out.Dgain = gains.D * in.headingrate_deg_s;
    Real command = out.Pgain + out.Igain + out.Dgain;
    out.command_norm = apply_command_clamp(command, clamp_mode, command_limit);
    return out;
}

} // namespace pypilot_algorithms
