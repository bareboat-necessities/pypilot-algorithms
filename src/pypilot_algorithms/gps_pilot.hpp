#pragma once

#include "basic_pilot.hpp"

namespace pypilot_algorithms {

template<typename Real = float>
using GpsPilotGains = BasicPilotGains<Real>;

template<typename Real = float>
using GpsPilotInput = BasicPilotInput<Real>;

template<typename Real = float>
using GpsPilotOutput = BasicPilotOutput<Real>;

template<typename Real = float>
inline GpsPilotOutput<Real> compute_gps_pilot(const GpsPilotInput<Real>& in,
                                              const GpsPilotGains<Real>& gains,
                                              CommandClamp clamp_mode = CommandClamp::raw,
                                              Real command_limit = Real(1)) {
    return compute_basic_pilot(in, gains, clamp_mode, command_limit);
}

} // namespace pypilot_algorithms
