#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    assert(nearf(pypilot_lowpass(0.2f, 10.0f, 0.0f), 2.0f));
    assert(nearf(pypilot_heading_filter(0.2f, 350.0f, 10.0f), 6.0f));
    assert(nearf(pypilot_heading_filter(0.2f, 10.0f, 350.0f), 354.0f));
    assert(nearf(pypilot_heading_offset_filter(170.0f, -170.0f, 0.5f), -180.0f));
    assert(nearf(pypilot_heading_offset_align_or_filter(false, 0.0f, 40.0f, 0.01f), 40.0f));
    assert(nearf(pypilot_heading_offset_align_or_filter(true, 0.0f, 40.0f, 0.5f), 20.0f));
    assert(nearf(pypilot_gps_speed_filter(0.0f, 10.0f), 0.02f));
    assert(nearf(pypilot_gps_heading_offset_measurement(100.0f, 80.0f), 20.0f));
    assert(nearf(pypilot_wind_heading_offset_measurement(-20.0f, 80.0f), 60.0f));
    assert(nearf(pypilot_wind_filtered_speed(0.0f, 10.0f), 0.1f));
    float wf = pypilot_wind_filter_factor(0.1f, 10.0f);
    assert(wf > 0.0f);
    assert(nearf(pypilot_wind_filtered_direction(10.0f, true, 350.0f, 0.2f), 6.0f));
    assert(nearf(pypilot_true_wind_speed(5.0f, 10.0f, 0.0f), 5.0f));
    assert(nearf(pypilot_true_wind_direction(5.0f, 10.0f, 0.0f), 0.0f));
    assert(nearf(pypilot_leeway_deg(10.0f, 5.0f), 2.0f));
    assert(pypilot_source_is_stale(1ULL, 0ULL));
    assert(pypilot_source_is_stale(9000001ULL, 1ULL));
    assert(!pypilot_source_is_stale(9000000ULL, 1ULL));
    assert(nearf(pypilot_heading_error(100.0f, 80.0f, false), 20.0f));
    assert(nearf(pypilot_heading_error(100.0f, 80.0f, true), -20.0f));
    assert(nearf(pypilot_heading_error(200.0f, 80.0f, false), 30.0f));
    assert(nearf(pypilot_heading_command_rate(0.0f, 100.0f, 90.0f, false), -1.0f));
    assert(nearf(pypilot_heading_command_rate(0.0f, 100.0f, 90.0f, true), 1.0f));
    assert(nearf(pypilot_heading_error_integral(0.0f, 25.0f, 2.0f), 0.5f));
    assert(nearf(pypilot_heading_error_integral(5.0f, 25.0f, 0.0f), 0.0f));
    assert(nearf(pypilot_decay_integral_by_command_rate(5.0f, 1.0f), 2.0f));
    assert(nearf(pypilot_decay_integral_by_command_rate(-5.0f, 1.0f), -2.0f));
    return 0;
}
