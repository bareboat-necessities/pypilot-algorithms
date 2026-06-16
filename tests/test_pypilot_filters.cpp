#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    assert(nearf(pypilot_lowpass(0.2f, 10.0f, 0.0f), 2.0f));
    assert(nearf(pypilot_heading_filter(0.2f, 350.0f, 10.0f), 6.0f));
    assert(nearf(pypilot_heading_filter(0.2f, 10.0f, 350.0f), 354.0f));
    assert(nearf(pypilot_heading_error(100.0f, 80.0f, false), 20.0f));
    assert(nearf(pypilot_heading_error(100.0f, 80.0f, true), -20.0f));
    assert(nearf(pypilot_heading_error(200.0f, 80.0f, false), 30.0f));
    assert(nearf(pypilot_heading_command_rate(0.0f, 100.0f, 90.0f, false), -1.0f));
    assert(nearf(pypilot_heading_command_rate(0.0f, 100.0f, 90.0f, true), 1.0f));
    assert(nearf(pypilot_heading_error_integral(0.0f, 25.0f, 2.0f), 0.5f));
    assert(nearf(pypilot_decay_integral_by_command_rate(5.0f, 1.0f), 2.0f));
    assert(nearf(pypilot_decay_integral_by_command_rate(-5.0f, 1.0f), -2.0f));
    return 0;
}
