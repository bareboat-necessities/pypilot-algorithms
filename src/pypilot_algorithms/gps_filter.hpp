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

    GpsFilterOutput()
        : valid(false), latitude_deg(Real(0)), longitude_deg(Real(0)),
          speed_kn(Real(0)), track_deg(Real(0)), timestamp_s(Real(0)) {}
};

template<typename Real = float, unsigned HistorySize = 32>
class GpsFilter2D {
public:
    GpsFilter2D()
        : initialized_(false), has_origin_(false), predict_time_s_(Real(0)),
          gps_system_time_offset_s_(Real(0)), gps_time_offset_s_(Real(0.7)),
          stale_count_(0), history_count_(0), history_next_(0), logger_(0) {
        set_default_noise();
        reset();
    }

    void set_logger(pypilot_syslib::Logger* logger) { logger_ = logger; }
    pypilot_syslib::Logger* logger() const { return logger_; }

    void reset() {
        initialized_ = false;
        has_origin_ = false;
        history_count_ = 0;
        history_next_ = 0;
        for (int i = 0; i < 4; ++i) {
            x_[i] = Real(0);
            for (int j = 0; j < 4; ++j) {
                p_[i][j] = (i == j) ? Real(1) : Real(0);
            }
        }
        output_ = GpsFilterOutput<Real>();
        log_reset();
    }

    bool initialized() const { return initialized_; }
    const GpsFilterOutput<Real>& output() const { return output_; }

    void set_gps_time_offset_s(Real value) { gps_time_offset_s_ = value; }
    Real gps_time_offset_s() const { return gps_time_offset_s_; }

    bool predict(const GpsFilterPrediction<Real>& prediction) {
        const Real dt_raw = prediction.time_s - predict_time_s_;
        predict_time_s_ = prediction.time_s;

        if (dt_raw < Real(0) || dt_raw > Real(0.5)) {
            log_prediction_reset(dt_raw);
            reset();
            return false;
        }
        if (!initialized_) return false;

        const Real u[2] = { prediction.accel_x_m_s2, prediction.accel_y_m_s2 };
        apply_prediction(dt_raw, u);
        push_history(prediction.time_s, clamp_dt(dt_raw), u);
        update_output(prediction.time_s);
        return true;
    }

    bool update(const GpsFilterFix<Real>& fix, Real now_s) {
        if (!fix.has_lat_lon || !fix.has_speed || !fix.has_track) return false;

        Real dt = now_s - fix.timestamp_s + gps_system_time_offset_s_;
        if (dt > Real(5)) {
            ++stale_count_;
            if (stale_count_ > 5) {
                gps_system_time_offset_s_ = fix.timestamp_s - now_s;
                reset();
            }
        } else {
            stale_count_ = 0;
            if (dt < Real(0)) {
                gps_system_time_offset_s_ = fix.timestamp_s - now_s;
                reset();
            }
        }

        const Real system_timestamp_s = fix.timestamp_s - gps_system_time_offset_s_ - gps_time_offset_s_;
        relocate_origin(fix.latitude_deg, fix.longitude_deg);

        Real z[4];
        z[0] = Real(0);
        z[1] = Real(0);
        const Real speed_m_s = fix.speed_kn / Real(1.944);
        const Real track_rad = fix.track_deg * Real(3.14159265358979323846 / 180.0);
        z[2] = speed_m_s * Real(sin(track_rad));
        z[3] = speed_m_s * Real(cos(track_rad));

        unsigned replay_count = rewind_to(system_timestamp_s);

        if (!initialized_) {
            for (int i = 0; i < 4; ++i) x_[i] = z[i];
            initialized_ = true;
            predict_time_s_ = fix.timestamp_s;
            update_output(now_s);
            return true;
        }

        if (!kalman_update(z)) return false;
        replay_predictions(replay_count);
        trim_history(replay_count);
        update_output(now_s);
        return output_.valid;
    }

private:
    struct HistoryEntry {
        Real time_s;
        Real dt_s;
        Real u[2];
        Real x[4];
        Real p[4][4];
    };

    bool initialized_;
    bool has_origin_;
    Real origin_lat_deg_;
    Real origin_lon_deg_;
    Real predict_time_s_;
    Real gps_system_time_offset_s_;
    Real gps_time_offset_s_;
    unsigned stale_count_;
    Real x_[4];
    Real p_[4][4];
    Real q_[4][4];
    Real r_[4][4];
    HistoryEntry history_[HistorySize];
    unsigned history_count_;
    unsigned history_next_;
    GpsFilterOutput<Real> output_;
    pypilot_syslib::Logger* logger_;

    void log_reset() const {
        pypilot_syslib::log_if(logger_, 0ULL,
                               pypilot_syslib::LogLevel::Info,
                               pypilot_syslib::LogModule::Algorithms,
                               pypilot_syslib::LogEvent::GpsFilterReset,
                               "gps filter reset");
    }

    void log_prediction_reset(Real dt_raw) const {
        pypilot_syslib::log_if(logger_, 0ULL,
                               pypilot_syslib::LogLevel::Warn,
                               pypilot_syslib::LogModule::Algorithms,
                               pypilot_syslib::LogEvent::GpsFilterPredictionReset,
                               "gps filter prediction reset",
                               0,
                               static_cast<float>(dt_raw));
    }

    void set_default_noise() {
        const Real pos_sigma = Real(10);
        const Real vel_sigma = Real(0.25);
        const Real pos_dev = Real(30);
        const Real vel_dev = Real(3);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                q_[i][j] = Real(0);
                r_[i][j] = Real(0);
            }
        }
        q_[0][0] = pos_dev * pos_dev;
        q_[1][1] = pos_dev * pos_dev;
        q_[2][2] = vel_dev * vel_dev;
        q_[3][3] = vel_dev * vel_dev;
        q_[0][2] = q_[2][0] = pos_dev * vel_dev;
        q_[1][3] = q_[3][1] = pos_dev * vel_dev;
        r_[0][0] = pos_sigma;
        r_[1][1] = pos_sigma;
        r_[2][2] = vel_sigma;
        r_[3][3] = vel_sigma;
    }

    static Real clamp_dt(Real dt) {
        if (dt < Real(0.02)) return Real(0.02);
        if (dt > Real(0.1)) return Real(0.1);
        return dt;
    }

    void apply_prediction(Real dt_in, const Real u[2]) {
        const Real dt = clamp_dt(dt_in);
        const Real dt2 = dt * dt / Real(2);

        x_[0] += dt * x_[2] + dt2 * u[0];
        x_[1] += dt * x_[3] + dt2 * u[1];
        x_[2] += dt * u[0];
        x_[3] += dt * u[1];

        Real fp[4][4];
        for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) fp[i][j] = p_[i][j];
        for (int j = 0; j < 4; ++j) {
            fp[0][j] += dt * p_[2][j];
            fp[1][j] += dt * p_[3][j];
        }

        Real out[4][4];
        for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) out[i][j] = fp[i][j];
        for (int i = 0; i < 4; ++i) {
            out[i][0] += dt * fp[i][2];
            out[i][1] += dt * fp[i][3];
        }
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) p_[i][j] = out[i][j] + q_[i][j];
        }
    }

    void relocate_origin(Real lat_deg, Real lon_deg) {
        if (initialized_ && has_origin_) {
            const GpsLatLon<Real> previous_ll = xy_to_ll(x_[0], x_[1], origin_lat_deg_, origin_lon_deg_);
            const GpsXY<Real> new_xy = ll_to_xy(previous_ll.latitude_deg, previous_ll.longitude_deg, lat_deg, lon_deg);
            x_[0] = new_xy.x_m;
            x_[1] = new_xy.y_m;
        }
        origin_lat_deg_ = lat_deg;
        origin_lon_deg_ = lon_deg;
        has_origin_ = true;
    }

    unsigned rewind_to(Real target_time_s) {
        unsigned replay_count = 0;
        for (unsigned i = 0; i < history_count_; ++i) {
            const unsigned idx = (history_next_ + HistorySize - history_count_ + i) % HistorySize;
            if (history_[idx].time_s >= target_time_s) {
                for (int a = 0; a < 4; ++a) {
                    x_[a] = history_[idx].x[a];
                    for (int b = 0; b < 4; ++b) p_[a][b] = history_[idx].p[a][b];
                }
                replay_count = history_count_ - i;
                history_count_ = i;
                return replay_count;
            }
        }
        return 0;
    }

    void replay_predictions(unsigned replay_count) {
        for (unsigned i = replay_count; i > 0; --i) {
            const unsigned idx = (history_next_ + HistorySize - i) % HistorySize;
            apply_prediction(history_[idx].dt_s, history_[idx].u);
        }
    }

    void trim_history(unsigned replay_count) {
        if (replay_count > history_count_) return;
        history_count_ -= replay_count;
        history_next_ = (history_next_ + HistorySize - replay_count) % HistorySize;
    }

    void push_history(Real time_s, Real dt_s, const Real u[2]) {
        HistoryEntry& e = history_[history_next_];
        e.time_s = time_s;
        e.dt_s = dt_s;
        e.u[0] = u[0];
        e.u[1] = u[1];
        for (int i = 0; i < 4; ++i) {
            e.x[i] = x_[i];
            for (int j = 0; j < 4; ++j) e.p[i][j] = p_[i][j];
        }
        history_next_ = (history_next_ + 1) % HistorySize;
        if (history_count_ < HistorySize) ++history_count_;
    }

    bool kalman_update(const Real z[4]) {
        Real y[4];
        for (int i = 0; i < 4; ++i) y[i] = z[i] - x_[i];

        for (int i = 0; i < 4; ++i) {
            const Real s = p_[i][i] + r_[i][i];
            if (s <= Real(0)) return false;
            const Real k = p_[i][i] / s;
            x_[i] += k * y[i];
            p_[i][i] *= (Real(1) - k);
        }
        return true;
    }

    void update_output(Real timestamp_s) {
        output_.valid = initialized_ && has_origin_;
        if (!output_.valid) return;
        const GpsLatLon<Real> ll = xy_to_ll(x_[0], x_[1], origin_lat_deg_, origin_lon_deg_);
        output_.latitude_deg = ll.latitude_deg;
        output_.longitude_deg = ll.longitude_deg;
        const Real speed_m_s = Real(sqrt(x_[2] * x_[2] + x_[3] * x_[3]));
        output_.speed_kn = speed_m_s * Real(1.944);
        output_.track_deg = Real(atan2(x_[2], x_[3]) * Real(180.0 / 3.14159265358979323846));
        if (output_.track_deg < Real(0)) output_.track_deg += Real(360);
        output_.timestamp_s = timestamp_s;
    }
};

} // namespace gps
} // namespace pypilot_algorithms
