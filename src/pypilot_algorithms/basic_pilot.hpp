#pragma once

#include "control.hpp"

namespace pypilot_algorithms {

template<typename Real = float>
struct BasicPilotGains {
    Real P = Real(0);
    Real D = Real(0);
    Real DD = Real(0);
    Real PR = Real(0);
    Real FF = Real(0);
};

template<typename Real = float>
struct BasicPilotInput {
    Real heading_error_deg = Real(0);
    Real headingrate_deg_s = Real(0);
    Real headingraterate_deg_s2 = Real(0);
    Real heading_command_rate_deg_s = Real(0);
};

template<typename Real = float>
struct BasicPilotOutput {
    Real command_norm = Real(0);
    Real Pgain = Real(0);
    Real Dgain = Real(0);
    Real DDgain = Real(0);
    Real PRgain = Real(0);
    Real FFgain = Real(0);
};

template<typename Real = float>
inline BasicPilotOutput<Real> compute_basic_pilot(const BasicPilotInput<Real>& in,
                                                  const BasicPilotGains<Real>& gains,
                                                  CommandClamp clamp_mode,
                                                  Real command_limit = Real(1)) {
    BasicPilotOutput<Real> out;
    out.Pgain = gains.P * in.heading_error_deg;
    out.Dgain = gains.D * in.headingrate_deg_s;
    out.DDgain = gains.DD * in.headingraterate_deg_s2;
    out.PRgain = gains.PR * signed_sqrt_abs(in.heading_error_deg);
    out.FFgain = gains.FF * in.heading_command_rate_deg_s;
    Real command = out.Pgain + out.Dgain + out.DDgain + out.PRgain + out.FFgain;
    out.command_norm = apply_command_clamp(command, clamp_mode, command_limit);
    return out;
}

template<typename Real = float>
inline BasicPilotOutput<Real> compute_basic_pilot(const BasicPilotInput<Real>& in,
                                                  const BasicPilotGains<Real>& gains,
                                                  Real command_limit = Real(1)) {
    return compute_basic_pilot(in, gains, CommandClamp::clamped, command_limit);
}

template<typename Real = float>
inline BasicPilotOutput<Real> compute_basic_pilot_raw(const BasicPilotInput<Real>& in,
                                                      const BasicPilotGains<Real>& gains) {
    return compute_basic_pilot(in, gains, CommandClamp::raw);
}

} // namespace pypilot_algorithms
