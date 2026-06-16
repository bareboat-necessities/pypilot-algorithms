#pragma once

#include <stdint.h>
#include "control.hpp"
#include "pypilot_filters.hpp"

namespace pypilot_algorithms {

enum PypilotServoFlag : uint32_t {
    pypilot_servo_sync_flag                   = 1u,
    pypilot_servo_overtemp_fault              = 2u,
    pypilot_servo_overcurrent_fault           = 4u,
    pypilot_servo_engaged_flag                = 8u,
    pypilot_servo_invalid_packet_flag         = 16u,
    pypilot_servo_port_pin_fault              = 32u,
    pypilot_servo_starboard_pin_fault         = 64u,
    pypilot_servo_bad_voltage_fault           = 128u,
    pypilot_servo_min_rudder_fault            = 256u,
    pypilot_servo_max_rudder_fault            = 512u,
    pypilot_servo_current_range_flag          = 1024u,
    pypilot_servo_bad_fuses_flag              = 2048u,
    pypilot_servo_rebooted_flag               = 32768u,
    pypilot_servo_port_overcurrent_fault      = 65536u,
    pypilot_servo_starboard_overcurrent_fault = 131072u,
    pypilot_servo_driver_timeout              = 262144u,
    pypilot_servo_saturated                   = 524288u
};

inline bool pypilot_servo_faulted(uint32_t flags) {
    return (flags & (pypilot_servo_overtemp_fault |
                     pypilot_servo_overcurrent_fault |
                     pypilot_servo_invalid_packet_flag |
                     pypilot_servo_port_pin_fault |
                     pypilot_servo_starboard_pin_fault |
                     pypilot_servo_bad_voltage_fault |
                     pypilot_servo_min_rudder_fault |
                     pypilot_servo_max_rudder_fault |
                     pypilot_servo_bad_fuses_flag |
                     pypilot_servo_port_overcurrent_fault |
                     pypilot_servo_starboard_overcurrent_fault |
                     pypilot_servo_driver_timeout)) != 0;
}

inline bool pypilot_servo_saturated_fault(uint32_t flags) {
    return (flags & pypilot_servo_saturated) != 0;
}

inline bool pypilot_servo_allows_command(uint32_t flags) {
    return !pypilot_servo_faulted(flags) && !pypilot_servo_saturated_fault(flags);
}

template<typename Real = float>
inline bool pypilot_rudder_feedback_valid(bool angle_valid,
                                          bool angle_live,
                                          Real range_deg) {
    return angle_valid && angle_live && range_deg > Real(0);
}

template<typename Real = float>
inline Real pypilot_rudder_position_command(Real command_norm, Real range_deg) {
    return command_norm * range_deg;
}

template<typename Real = float>
inline Real pypilot_reset_command() {
    return Real(0);
}

} // namespace pypilot_algorithms
