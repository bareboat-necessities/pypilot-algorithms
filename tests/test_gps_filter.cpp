#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>

static bool near(double a, double b, double eps) { return std::fabs(a - b) < eps; }

int main() {
    typedef pypilot_algorithms::gps::GpsFilter2D<double> Filter;
    typedef pypilot_algorithms::gps::GpsFilterFix<double> Fix;
    typedef pypilot_algorithms::gps::GpsFilterPrediction<double> Prediction;

    Filter filter;
    assert(!filter.initialized());

    Fix fix;
    fix.has_lat_lon = true;
    fix.latitude_deg = 40.0;
    fix.longitude_deg = -74.0;
    fix.has_speed = true;
    fix.speed_kn = 10.0;
    fix.has_track = true;
    fix.track_deg = 90.0;
    fix.timestamp_s = 100.0;

    assert(filter.update(fix, 100.7));
    assert(filter.initialized());
    assert(filter.output().valid);
    assert(near(filter.output().latitude_deg, 40.0, 1e-9));
    assert(near(filter.output().longitude_deg, -74.0, 1e-9));
    assert(near(filter.output().speed_kn, 10.0 * (1.94 / 1.944), 1e-6));
    assert(near(filter.output().track_deg, 90.0, 1e-6));

    Prediction prediction;
    prediction.time_s = 100.2;
    prediction.accel_x_m_s2 = 0.0;
    prediction.accel_y_m_s2 = 0.0;
    assert(filter.predict(prediction));
    assert(filter.output().valid);
    assert(filter.output().longitude_deg > -74.0);
    assert(near(filter.output().latitude_deg, 40.0, 1e-5));

    Fix moved = fix;
    moved.longitude_deg = -73.999;
    moved.timestamp_s = 100.2;
    assert(filter.update(moved, 100.9));
    assert(filter.output().valid);
    assert(filter.output().longitude_deg > -74.0);

    Filter stale;
    assert(stale.update(fix, 110.0));
    for (int i = 0; i < 6; ++i) {
        assert(stale.update(fix, 110.0 + i));
    }
    assert(stale.initialized());

    return 0;
}
