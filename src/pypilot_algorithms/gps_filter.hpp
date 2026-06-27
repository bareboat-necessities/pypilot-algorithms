#pragma once

#include <stdint.h>
#include <math.h>
#if defined(__has_include)
#  if __has_include(<pypilot_syslib.hpp>)
#    include <pypilot_syslib.hpp>
#    define PYPILOT_ALGORITHMS_HAVE_SYSLIB 1
#  endif
#endif
#ifndef PYPILOT_ALGORITHMS_HAVE_SYSLIB
#ifndef PYPILOT_SYSLIB_COMPAT_FALLBACK_HPP
#define PYPILOT_SYSLIB_COMPAT_FALLBACK_HPP
namespace pypilot_syslib {
class Logger {};
enum class LogLevel { Debug, Info, Warn, Error, Critical };
enum class LogModule { Algorithms, DataModel, Sensors, PilotsLogic, GpsAdapter, SteeringSignaling, ServoProtocol, Nmea0183, SignalK, Runtime };
enum class LogEvent {
    SystemStartup,
    SystemShutdown,
    ConfigurationLoaded,
    ConfigurationInvalid,
    SourceSelected,
    SourceRejected,
    SourceTimedOut,
    SensorSampleRejected,
    GpsFixAccepted,
    GpsFixRejected,
    GpsFilterReset,
    GpsFilterPredictionReset,
    WmmEvaluationInvalid,
    PilotModeChanged,
    PilotCommandComputed,
    ApbNavCommandAccepted,
    ApbNavCommandRateLimited,
    RudderCalibrationInvalid,
    ServoCommandBlocked,
    ServoFeedbackFault
};
static inline void log_if(Logger*, uint64_t, LogLevel, LogModule, LogEvent, const char*, int = 0, float = 0.0f) {}
}
#endif
#endif
#include "gps_math.hpp"

namespace pypilot_algorithms {
namespace gps {

template<typename Real = float>
struct GpsFilterFix {
    bool has_lat_lon;
    Real latitude_deg;
    Real longitude_deg;
    bool has_speed;
    Real speed_kn;
    bool has_track;
    Real track_deg;
    Real timestamp_s;

    GpsFilterFix()
        : has_lat_lon(false), latitude_deg(Real(0)), longitude_deg(Real(0)),
          has_speed(false), speed_kn(Real(0)), has_track(false), track_deg(Real(0)),
          timestamp_s(Real(0)) {}
};

template<typename Real = float>
struct GpsFilterPrediction {
    Real time_s;
    Real accel_x_m_s2;
    Real accel_y_m_s2;

    GpsFilterPrediction() : time_s(Real(0)), accel_x_m_s2(Real(0)), accel_y_m_s2(Real(0)) {}
};

template<typename Real = float>
struct GpsFilterOutput {
    bool valid;
    Real latitude_deg;
    Real longitude_deg;
    Real speed_kn;
    Real track_deg;
    Real timestamp_s;
};
