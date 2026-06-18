#pragma once

#include <cmath>

namespace pypilot_algorithms {
namespace gps {

static const double earth_radius_m = 6378137.0;
static const double earth_m_per_degree = earth_radius_m * 2.0 * 3.14159265358979323846 / 360.0;

template<typename Real = double>
struct GpsXY {
    Real x_m;
    Real y_m;
};

template<typename Real = double>
struct GpsLatLon {
    Real latitude_deg;
    Real longitude_deg;
};

template<typename Real>
inline Real wrap_180_degrees(Real value) {
    while (value > Real(180)) value -= Real(360);
    while (value <= Real(-180)) value += Real(360);
    return value;
}

template<typename Real>
inline GpsXY<Real> ll_to_xy(Real latitude_deg,
                            Real longitude_deg,
                            Real origin_latitude_deg,
                            Real origin_longitude_deg) {
    const Real cs = std::cos(origin_latitude_deg * Real(3.14159265358979323846 / 180.0));
    const Real yc = latitude_deg - origin_latitude_deg;
    const Real xc = wrap_180_degrees(longitude_deg - origin_longitude_deg);
    GpsXY<Real> out;
    out.x_m = Real(earth_m_per_degree) * xc * cs;
    out.y_m = Real(earth_m_per_degree) * yc;
    return out;
}

template<typename Real>
inline GpsLatLon<Real> xy_to_ll(Real x_m,
                                Real y_m,
                                Real origin_latitude_deg,
                                Real origin_longitude_deg) {
    const Real cs = std::cos(origin_latitude_deg * Real(3.14159265358979323846 / 180.0));
    GpsLatLon<Real> out;
    out.latitude_deg = y_m / Real(earth_m_per_degree) + origin_latitude_deg;
    out.longitude_deg = x_m / Real(earth_m_per_degree) / cs + origin_longitude_deg;
    return out;
}

inline double meters_per_second_to_knots(double value) {
    return value * 1.9438444924406048;
}

} // namespace gps
} // namespace pypilot_algorithms
