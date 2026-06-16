#pragma once

#include "control.hpp"

namespace pypilot_algorithms {

template<typename Real = float>
struct VmgPilotGains {
    Real P = Real(0);
    Real D = Real(0);
    Real DD = Real(0);
};

template<typename Real = float>
struct VmgPilotInput {
    Real heading_error_deg = Real(0);
    Real headingrate_deg_s = Real(0);
    Real headingraterate_deg_s2 = Real(0);
};

template<typename Real = float>
struct VmgPilotOutput {
    Real command_norm = Real(0);
    Real Pgain = Real(0);
    Real Dgain = Real(0);
    Real DDgain = Real(0);
};

template<typename Real = float>
inline VmgPilotOutput<Real> compute_vmg_pilot(const VmgPilotInput<Real>& in,
                                              const VmgPilotGains<Real>& gains,
                                              CommandClamp clamp_mode = CommandClamp::raw,
                                              Real command_limit = Real(1)) {
    VmgPilotOutput<Real> out;
    out.Pgain = gains.P * in.heading_error_deg;
    out.Dgain = gains.D * in.headingrate_deg_s;
    out.DDgain = gains.DD * in.headingraterate_deg_s2;
    Real command = out.Pgain + out.Dgain + out.DDgain;
    out.command_norm = apply_command_clamp(command, clamp_mode, command_limit);
    return out;
}

} // namespace pypilot_algorithms
