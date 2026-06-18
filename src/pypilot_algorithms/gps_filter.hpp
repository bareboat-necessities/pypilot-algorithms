#pragma once

#include <stdint.h>
#include <math.h>
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
          stale_count_(0), history_count_(0), history_next_(0) {
        set_default_noise();
        reset();
    }

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
    }

    bool initialized() const { return initialized_; }
    const GpsFilterOutput<Real>& output() const { return output_; }

    void set_gps_time_offset_s(Real value) { gps_time_offset_s_ = value; }
    Real gps_time_offset_s() const { return gps_time_offset_s_; }

    bool predict(const GpsFilterPrediction<Real>& prediction) {
        const Real dt_raw = prediction.time_s - predict_time_s_;
        predict_time_s_ = prediction.time_s;

        if (dt_raw < Real(0) || dt_raw > Real(0.5)) {
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
            update_output(now_s);
            return true;
        }

        kalman_update(z);
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

    void push_history(Real t, Real dt, const Real u[2]) {
        HistoryEntry& h = history_[history_next_];
        h.time_s = t;
        h.dt_s = dt;
        h.u[0] = u[0];
        h.u[1] = u[1];
        for (int i = 0; i < 4; ++i) {
            h.x[i] = x_[i];
            for (int j = 0; j < 4; ++j) h.p[i][j] = p_[i][j];
        }
        history_next_ = (history_next_ + 1U) % HistorySize;
        if (history_count_ < HistorySize) ++history_count_;
    }

    unsigned history_index_from_oldest(unsigned offset) const {
        const unsigned oldest = (history_next_ + HistorySize - history_count_) % HistorySize;
        return (oldest + offset) % HistorySize;
    }

    unsigned rewind_to(Real t) {
        unsigned replay_count = 0;
        for (unsigned n = 0; n < history_count_; ++n) {
            const unsigned offset = history_count_ - 1U - n;
            const unsigned idx = history_index_from_oldest(offset);
            const HistoryEntry& h = history_[idx];
            if (h.time_s < t) {
                for (int i = 0; i < 4; ++i) {
                    x_[i] = h.x[i];
                    for (int j = 0; j < 4; ++j) p_[i][j] = h.p[i][j];
                }
                replay_count = n;
                break;
            }
        }
        return replay_count;
    }

    void replay_predictions(unsigned replay_count) {
        if (replay_count == 0) return;
        for (unsigned n = replay_count; n > 0; --n) {
            const unsigned offset = history_count_ - n;
            const unsigned idx = history_index_from_oldest(offset);
            apply_prediction(history_[idx].dt_s, history_[idx].u);
        }
    }

    void trim_history(unsigned keep_newest) {
        if (keep_newest >= history_count_) return;
        history_count_ = keep_newest;
        history_next_ = keep_newest % HistorySize;
    }

    static bool invert4(const Real in[4][4], Real out[4][4]) {
        Real a[4][8];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) a[i][j] = in[i][j];
            for (int j = 0; j < 4; ++j) a[i][4 + j] = (i == j) ? Real(1) : Real(0);
        }
        for (int col = 0; col < 4; ++col) {
            int pivot = col;
            Real best = Real(fabs(a[col][col]));
            for (int row = col + 1; row < 4; ++row) {
                const Real v = Real(fabs(a[row][col]));
                if (v > best) { best = v; pivot = row; }
            }
            if (best < Real(1e-9)) return false;
            if (pivot != col) {
                for (int j = 0; j < 8; ++j) {
                    const Real tmp = a[col][j];
                    a[col][j] = a[pivot][j];
                    a[pivot][j] = tmp;
                }
            }
            const Real div = a[col][col];
            for (int j = 0; j < 8; ++j) a[col][j] /= div;
            for (int row = 0; row < 4; ++row) {
                if (row == col) continue;
                const Real f = a[row][col];
                for (int j = 0; j < 8; ++j) a[row][j] -= f * a[col][j];
            }
        }
        for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) out[i][j] = a[i][4 + j];
        return true;
    }

    bool kalman_update(const Real z[4]) {
        Real s[4][4];
        for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) s[i][j] = p_[i][j] + r_[i][j];
        Real inv_s[4][4];
        if (!invert4(s, inv_s)) return false;

        Real k[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                k[i][j] = Real(0);
                for (int m = 0; m < 4; ++m) k[i][j] += p_[i][m] * inv_s[m][j];
            }
        }

        Real y[4];
        for (int i = 0; i < 4; ++i) y[i] = z[i] - x_[i];
        for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) x_[i] += k[i][j] * y[j];

        Real new_p[4][4];
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                new_p[i][j] = p_[i][j];
                for (int m = 0; m < 4; ++m) new_p[i][j] -= k[i][m] * p_[m][j];
            }
        }
        for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) p_[i][j] = new_p[i][j];
        return true;
    }

    void update_output(Real timestamp_s) {
        if (!initialized_ || !has_origin_) {
            output_.valid = false;
            return;
        }
        const GpsLatLon<Real> ll = xy_to_ll(x_[0], x_[1], origin_lat_deg_, origin_lon_deg_);
        output_.valid = true;
        output_.latitude_deg = ll.latitude_deg;
        output_.longitude_deg = ll.longitude_deg;
        output_.speed_kn = Real(sqrt(x_[2] * x_[2] + x_[3] * x_[3]) * Real(1.94));
        output_.track_deg = wrap_180_degrees(Real(atan2(x_[2], x_[3]) * Real(180.0 / 3.14159265358979323846)));
        output_.timestamp_s = timestamp_s;
    }
};

} // namespace gps
} // namespace pypilot_algorithms
