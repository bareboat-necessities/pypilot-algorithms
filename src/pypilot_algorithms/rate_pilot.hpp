#pragma once

#include "control.hpp"

namespace pypilot_algorithms {

template<typename Real = float>
struct RatePilotGains {
    Real D = Real(0);
    Real DD = Real(0);
    Real FF = Real(0);
    Real max_turn_rate_deg_s = Real(2);
    Real turn_rate_rate_deg_s2 = Real(0.5);
};

template<typename Real = float>
struct RatePilotInput {
    Real heading_error_deg = Real(0);
    Real headingrate_deg_s = Real(0);
    Real headingraterate_deg_s2 = Real(0);
    Real heading_command_rate_deg_s = Real(0);
};

template<typename Real = float>
struct RatePilotOutput {
    Real command_norm = Real(0);
    Real target_rate_deg_s = Real(0);
    Real rate_error_deg_s = Real(0);
    Real Dgain = Real(0);
    Real DDgain = Real(0);
    Real FFgain = Real(0);
};

template<typename Real = float>
inline RatePilotOutput<Real> compute_rate_pilot(const RatePilotInput<Real>& in,
                                                const RatePilotGains<Real>& gains,
                                                CommandClamp clamp_mode = CommandClamp::raw,
                                                Real command_limit = Real(1)) {
    RatePilotOutput<Real> out;
    out.target_rate_deg_s = clamp(in.heading_error_deg * gains.turn_rate_rate_deg_s2,
                                  -gains.max_turn_rate_deg_s,
                                  gains.max_turn_rate_deg_s);
    out.rate_error_deg_s = in.headingrate_deg_s - out.target_rate_deg_s;
    out.Dgain = gains.D * out.rate_error_deg_s;
    out.DDgain = gains.DD * in.headingraterate_deg_s2;
    out.FFgain = gains.FF * in.heading_command_rate_deg_s;
    Real command = out.Dgain + out.DDgain + out.FFgain;
    out.command_norm = apply_command_clamp(command, clamp_mode, command_limit);
    return out;
}

} // namespace pypilot_algorithms
