#pragma once

#include "angles.hpp"
#include "control.hpp"

namespace pypilot_algorithms {

template<typename Real = float>
inline Real pypilot_lowpass(Real alpha, Real sample, Real previous) {
    alpha = clamp(alpha, Real(0), Real(1));
    return alpha * sample + (Real(1) - alpha) * previous;
}

template<typename Real = float>
inline Real pypilot_heading_filter(Real alpha, Real sample_deg, Real previous_deg) {
    alpha = clamp(alpha, Real(0), Real(1));
    if (sample_deg - previous_deg > Real(180)) {
        sample_deg -= Real(360);
    } else if (previous_deg - sample_deg > Real(180)) {
        previous_deg -= Real(360);
    }

    Real result = alpha * sample_deg + (Real(1) - alpha) * previous_deg;
    if (result < Real(0)) {
        result += Real(360);
    }
    if (result >= Real(360)) {
        result -= Real(360);
    }
    return result;
}


template<typename Real = float>
inline Real pypilot_resolv_to(Real angle_deg, Real offset_deg) {
    while (offset_deg - angle_deg > Real(180)) {
        angle_deg += Real(360);
    }
    while (offset_deg - angle_deg <= Real(-180)) {
        angle_deg -= Real(360);
    }
    return angle_deg;
}

template<typename Real = float>
inline Real pypilot_resolv(Real angle_deg) {
    return pypilot_resolv_to(angle_deg, Real(0));
}

template<typename Real = float>
inline Real pypilot_heading_offset_filter(Real current_offset_deg,
                                          Real measured_offset_deg,
                                          Real alpha) {
    alpha = clamp(alpha, Real(0), Real(1));
    Real resolved_measured = pypilot_resolv_to(measured_offset_deg, current_offset_deg);
    return pypilot_resolv(alpha * resolved_measured + (Real(1) - alpha) * current_offset_deg);
}

template<typename Real = float>
inline Real pypilot_gps_speed_filter(Real previous_speed_kn,
                                     Real gps_speed_kn,
                                     Real alpha = Real(0.002)) {
    return pypilot_lowpass(alpha, gps_speed_kn, previous_speed_kn);
}

template<typename Real = float>
inline Real pypilot_gps_heading_offset_alpha(Real smoothed_gps_speed_kn) {
    if (smoothed_gps_speed_kn < Real(0)) {
        smoothed_gps_speed_kn = Real(0);
    }
    return Real(0.005) * static_cast<Real>(log(smoothed_gps_speed_kn + Real(1)));
}

template<typename Real = float>
inline Real pypilot_gps_heading_offset_measurement(Real gps_track_deg, Real compass_heading_deg) {
    return gps_track_deg - compass_heading_deg;
}

template<typename Real = float>
inline Real pypilot_wind_heading_offset_measurement(Real filtered_wind_direction_deg,
                                                    Real compass_heading_deg) {
    return filtered_wind_direction_deg + compass_heading_deg;
}

template<typename Real = float>
inline Real pypilot_heading_error(Real heading_deg,
                                  Real heading_command_deg,
                                  bool wind_mode,
                                  Real limit_deg = Real(30)) {
    Real error = clamp(wrap_180_deg(heading_deg - heading_command_deg), -limit_deg, limit_deg);
    return wind_mode ? -error : error;
}

template<typename Real = float>
inline Real pypilot_heading_command_rate(Real previous_rate_deg,
                                         Real heading_command_deg,
                                         Real last_heading_command_deg,
                                         bool wind_mode,
                                         Real alpha = Real(0.1)) {
    Real diff = wrap_180_deg(heading_command_deg - last_heading_command_deg);
    if (!wind_mode) {
        diff = -diff;
    }
    return pypilot_lowpass(alpha, diff, previous_rate_deg);
}

template<typename Real = float>
inline Real pypilot_heading_error_integral(Real previous_integral_deg,
                                           Real heading_error_deg,
                                           Real dt_s,
                                           Real limit_deg = Real(10)) {
    if (dt_s < Real(0)) {
        dt_s = Real(0);
    }
    if (dt_s > Real(1)) {
        dt_s = Real(1);
    }
    return clamp(previous_integral_deg + (heading_error_deg / Real(50)) * dt_s,
                 -limit_deg,
                 limit_deg);
}

template<typename Real = float>
inline Real pypilot_decay_integral_by_command_rate(Real integral_deg,
                                                   Real command_rate_deg_s) {
    if (integral_deg == Real(0)) {
        return Real(0);
    }
    Real sign_value = integral_deg > Real(0) ? Real(1) : Real(-1);
    Real magnitude = abs_real(integral_deg) - abs_real(command_rate_deg_s) * Real(3);
    if (magnitude < Real(0)) {
        magnitude = Real(0);
    }
    return sign_value * magnitude;
}

} // namespace pypilot_algorithms
