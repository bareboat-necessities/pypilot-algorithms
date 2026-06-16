#pragma once

#include <math.h>
#include "angles.hpp"

namespace pypilot_algorithms {

template<typename Real = float>
struct GainContribution {
    Real output = Real(0);
    Real contribution = Real(0);
};

enum class CommandClamp {
    raw,
    clamped
};

template<typename Real>
inline Real sign(Real value) {
    return value > Real(0) ? Real(1) : (value < Real(0) ? Real(-1) : Real(0));
}

template<typename Real>
inline Real signed_sqrt_abs(Real value) {
    return sign(value) * static_cast<Real>(sqrt(abs_real(value)));
}

template<typename Real>
inline GainContribution<Real> apply_gain(Real gain, Real input) {
    GainContribution<Real> out;
    out.contribution = gain * input;
    out.output = out.contribution;
    return out;
}

template<typename Real>
inline Real clamp_command(Real command, Real limit = Real(1)) {
    return minmax(command, limit);
}

template<typename Real>
inline Real apply_command_clamp(Real command, CommandClamp clamp_mode, Real limit = Real(1)) {
    return clamp_mode == CommandClamp::clamped ? clamp_command(command, limit) : command;
}

} // namespace pypilot_algorithms
