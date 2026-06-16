#pragma once

#include <math.h>

namespace pypilot_algorithms {

template<typename Real>
inline Real abs_real(Real v) { return v < Real(0) ? -v : v; }

template<typename Real>
inline Real min_real(Real a, Real b) { return a < b ? a : b; }

template<typename Real>
inline Real max_real(Real a, Real b) { return a > b ? a : b; }

template<typename Real>
inline Real clamp(Real value, Real lo, Real hi) {
    return value < lo ? lo : (value > hi ? hi : value);
}

template<typename Real>
inline Real minmax(Real value, Real limit) {
    Real l = abs_real(limit);
    return clamp(value, -l, l);
}

template<typename Real>
inline Real wrap_360_deg(Real angle_deg) {
    while (angle_deg >= Real(360)) angle_deg -= Real(360);
    while (angle_deg < Real(0)) angle_deg += Real(360);
    return angle_deg;
}

template<typename Real>
inline Real wrap_180_deg(Real angle_deg) {
    angle_deg = wrap_360_deg(angle_deg + Real(180)) - Real(180);
    if (angle_deg <= Real(-180)) angle_deg += Real(360);
    return angle_deg;
}

template<typename Real>
inline Real resolv_deg(Real angle_deg) {
    return wrap_180_deg(angle_deg);
}

// pypilot convention: error is measured heading minus command, then limited.
template<typename Real>
inline Real heading_error_deg(Real heading_deg, Real command_deg, Real limit_deg = Real(30)) {
    return minmax(wrap_180_deg(heading_deg - command_deg), limit_deg);
}

template<typename Real>
inline Real command_error_deg(Real command_deg, Real heading_deg, Real limit_deg = Real(30)) {
    return minmax(wrap_180_deg(command_deg - heading_deg), limit_deg);
}

template<typename Real>
inline Real angle_add_deg(Real a, Real delta) {
    return wrap_180_deg(a + delta);
}

template<typename Real>
inline Real angle_delta_deg(Real from_deg, Real to_deg) {
    return wrap_180_deg(to_deg - from_deg);
}

} // namespace pypilot_algorithms
