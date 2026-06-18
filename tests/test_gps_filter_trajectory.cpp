#include <cassert>
#include <cmath>
#include <iostream>
#include <pypilot_algorithms/gps_filter.hpp>
#include <pypilot_algorithms/gps_math.hpp>

namespace gps = pypilot_algorithms::gps;

struct ReferencePoint {
    double t;
    double x_m;
    double y_m;
    double vx_m_s;
    double vy_m_s;
    double speed_kn;
    double track_deg;
    double latitude_deg;
    double longitude_deg;
};

class DeterministicNoise {
public:
    explicit DeterministicNoise(unsigned seed) : state_(seed) {}

    double uniform01() {
        state_ = 1664525U * state_ + 1013904223U;
        return double((state_ >> 8) & 0x00ffffffU) / double(0x01000000U);
    }

    double normal() {
        double s = 0.0;
        for (int i = 0; i < 12; ++i) s += uniform01();
        return s - 6.0;
    }

private:
    unsigned state_;
};

static double deg_to_rad(double deg) { return deg * 3.14159265358979323846 / 180.0; }
static double rad_to_deg(double rad) { return rad * 180.0 / 3.14159265358979323846; }

static double wrap_360(double deg) {
    while (deg < 0.0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    return deg;
}

static double speed_kn_at(double t) {
    double gust = (t > 70.0 && t < 115.0) ? 0.9 : 0.0;
    return 5.2 + 0.8 * std::sin(0.035 * t) + 0.35 * std::sin(0.11 * t) + gust;
}

static double track_deg_at(double t) {
    return wrap_360(55.0 + 18.0 * std::sin(0.018 * t) + 7.0 * std::sin(0.071 * t));
}

static ReferencePoint make_reference(double t,
                                     double x_m,
                                     double y_m,
                                     double origin_lat,
                                     double origin_lon) {
    const double speed_kn = speed_kn_at(t);
    const double speed_m_s = speed_kn / 1.944;
    const double track_deg = track_deg_at(t);
    const double track_rad = deg_to_rad(track_deg);
    const double vx = speed_m_s * std::sin(track_rad);
    const double vy = speed_m_s * std::cos(track_rad);
    const auto ll = gps::xy_to_ll<double>(x_m, y_m, origin_lat, origin_lon);

    ReferencePoint r;
    r.t = t;
    r.x_m = x_m;
    r.y_m = y_m;
    r.vx_m_s = vx;
    r.vy_m_s = vy;
    r.speed_kn = speed_kn;
    r.track_deg = track_deg;
    r.latitude_deg = ll.latitude_deg;
    r.longitude_deg = ll.longitude_deg;
    return r;
}

int main() {
    typedef gps::GpsFilter2D<double> Filter;
    typedef gps::GpsFilterFix<double> Fix;
    typedef gps::GpsFilterPrediction<double> Prediction;

    static const int Steps = 900;
    static const double Dt = 0.2;
    static const double StartTime = 100.0;
    static const double OriginLat = 40.0;
    static const double OriginLon = -74.0;
    static const double PosNoiseSigmaM = 3.0;
    static const double SpeedNoiseSigmaKn = 0.22;
    static const double TrackNoiseSigmaDeg = 2.0;
    static const double WarmupS = 30.0;

    ReferencePoint ref[Steps + 1];
    double x = 0.0;
    double y = 0.0;
    for (int k = 0; k <= Steps; ++k) {
        const double t = StartTime + k * Dt;
        ref[k] = make_reference(t - StartTime, x, y, OriginLat, OriginLon);
        if (k < Steps) {
            x += ref[k].vx_m_s * Dt;
            y += ref[k].vy_m_s * Dt;
        }
    }

    DeterministicNoise noise(0x5a17b0a7U);
    Filter filter;

    auto make_fix = [&](int k) {
        const double noisy_x = ref[k].x_m + PosNoiseSigmaM * noise.normal();
        const double noisy_y = ref[k].y_m + PosNoiseSigmaM * noise.normal();
        const auto noisy_ll = gps::xy_to_ll<double>(noisy_x, noisy_y, OriginLat, OriginLon);

        Fix fix;
        fix.has_lat_lon = true;
        fix.latitude_deg = noisy_ll.latitude_deg;
        fix.longitude_deg = noisy_ll.longitude_deg;
        fix.has_speed = true;
        fix.speed_kn = ref[k].speed_kn + SpeedNoiseSigmaKn * noise.normal();
        fix.has_track = true;
        fix.track_deg = wrap_360(ref[k].track_deg + TrackNoiseSigmaDeg * noise.normal());
        fix.timestamp_s = ref[k].t;
        return fix;
    };

    assert(filter.update(make_fix(0), ref[0].t + 0.7));

    double pos_err2_sum = 0.0;
    double speed_err2_sum = 0.0;
    int count = 0;

    for (int k = 1; k <= Steps; ++k) {
        Prediction prediction;
        prediction.time_s = ref[k].t;
        prediction.accel_x_m_s2 = (ref[k].vx_m_s - ref[k - 1].vx_m_s) / Dt;
        prediction.accel_y_m_s2 = (ref[k].vy_m_s - ref[k - 1].vy_m_s) / Dt;
        assert(filter.predict(prediction));

        if ((k % 5) == 0) {
            assert(filter.update(make_fix(k), ref[k].t + 0.7));
        }

        if (ref[k].t - StartTime >= WarmupS) {
            const auto& out = filter.output();
            assert(out.valid);
            const auto out_xy = gps::ll_to_xy<double>(out.latitude_deg, out.longitude_deg, OriginLat, OriginLon);
            const double dx = out_xy.x_m - ref[k].x_m;
            const double dy = out_xy.y_m - ref[k].y_m;
            const double ds = out.speed_kn - ref[k].speed_kn;
            pos_err2_sum += dx * dx + dy * dy;
            speed_err2_sum += ds * ds;
            ++count;
        }
    }

    assert(count > 0);
    const double pos_rms_m = std::sqrt(pos_err2_sum / count);
    const double speed_rms_kn = std::sqrt(speed_err2_sum / count);

    if (!(pos_rms_m < 25.0 && speed_rms_kn < 1.2)) {
        std::cerr << "gps filter trajectory RMS failed: pos_rms_m=" << pos_rms_m
                  << " speed_rms_kn=" << speed_rms_kn << "\n";
        return 1;
    }

    return 0;
}
