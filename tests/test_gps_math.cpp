#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>

static bool near(double a, double b, double eps) { return std::fabs(a - b) < eps; }

int main() {
    const double lat0 = 40.0;
    const double lon0 = -74.0;
    const double lat = 40.001;
    const double lon = -73.998;

    const auto xy = pypilot_algorithms::gps::ll_to_xy(lat, lon, lat0, lon0);
    assert(std::fabs(xy.x_m) > 100.0);
    assert(std::fabs(xy.y_m) > 100.0);

    const auto ll = pypilot_algorithms::gps::xy_to_ll(xy.x_m, xy.y_m, lat0, lon0);
    assert(near(ll.latitude_deg, lat, 1e-9));
    assert(near(ll.longitude_deg, lon, 1e-9));

    const double dateline_lat0 = 10.0;
    const double dateline_lon0 = 179.9;
    const double dateline_lat = 10.0;
    const double dateline_lon = -179.9;
    const auto dateline_xy = pypilot_algorithms::gps::ll_to_xy(dateline_lat,
                                                                dateline_lon,
                                                                dateline_lat0,
                                                                dateline_lon0);
    const double expected_x = pypilot_algorithms::gps::earth_m_per_degree * 0.2 *
                              std::cos(dateline_lat0 * 3.14159265358979323846 / 180.0);
    assert(near(dateline_xy.x_m, expected_x, 1e-6));
    assert(near(dateline_xy.y_m, 0.0, 1e-9));

    const auto dateline_ll = pypilot_algorithms::gps::xy_to_ll(dateline_xy.x_m,
                                                                dateline_xy.y_m,
                                                                dateline_lat0,
                                                                dateline_lon0);
    assert(near(dateline_ll.latitude_deg, dateline_lat, 1e-9));
    assert(near(pypilot_algorithms::gps::wrap_180_degrees(dateline_ll.longitude_deg - dateline_lon), 0.0, 1e-9));

    assert(near(pypilot_algorithms::gps::wrap_180_degrees(181.0), -179.0, 1e-12));
    assert(near(pypilot_algorithms::gps::wrap_180_degrees(-181.0), 179.0, 1e-12));
    assert(near(pypilot_algorithms::gps::meters_per_second_to_knots(1.0), 1.9438444924406048, 1e-12));

    return 0;
}
