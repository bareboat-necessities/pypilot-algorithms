#pragma once

#include <stdint.h>
#include "pypilot_filters.hpp"

namespace pypilot_algorithms {

enum class PypilotSource : uint8_t {
    gpsd,
    servo,
    serial,
    tcp,
    signalk,
    water_wind,
    gps_wind,
    none
};

inline int pypilot_source_priority(PypilotSource source) {
    switch (source) {
    case PypilotSource::gpsd:       return 1;
    case PypilotSource::servo:      return 1;
    case PypilotSource::serial:     return 2;
    case PypilotSource::tcp:        return 3;
    case PypilotSource::signalk:    return 4;
    case PypilotSource::water_wind: return 5;
    case PypilotSource::gps_wind:   return 6;
    case PypilotSource::none:       return 7;
    }
    return 7;
}

inline bool pypilot_source_is_none(PypilotSource source) {
    return source == PypilotSource::none;
}

inline bool pypilot_source_can_update(PypilotSource current_source,
                                      PypilotSource incoming_source,
                                      bool current_source_live = true) {
    if (incoming_source == PypilotSource::none) {
        return true;
    }
    if (current_source == PypilotSource::none || !current_source_live) {
        return true;
    }
    return pypilot_source_priority(incoming_source) <= pypilot_source_priority(current_source);
}

inline bool pypilot_source_can_update(PypilotSource current_source,
                                      PypilotSource incoming_source,
                                      uint64_t now_us,
                                      uint64_t current_last_update_us,
                                      uint64_t timeout_us = 8000000ULL) {
    bool live = current_source != PypilotSource::none &&
                !pypilot_source_is_stale(now_us, current_last_update_us, timeout_us);
    return pypilot_source_can_update(current_source, incoming_source, live);
}

} // namespace pypilot_algorithms
