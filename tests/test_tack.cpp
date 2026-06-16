#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    PypilotTackConfig<> config;
    config.angle_deg = 100.0f;
    config.rate_deg_s = 15.0f;
    config.threshold_percent = 50.0f;

    PypilotTackInput<> input;
    input.direction = PypilotTackDirection::port;
    input.heading_command_deg = 100.0f;
    input.heading_deg = 80.0f;
    input.headingrate_deg_s = 0.0f;
    input.headingraterate_deg_s2 = 0.0f;

    PypilotTackOutput<> output = pypilot_tack_compute(input, config);
    assert(output.override_pilot);
    assert(!output.completed);
    assert(nearf(output.command_norm, 1.0f));
    assert(nearf(output.new_heading_command_deg, 0.0f));

    input.heading_deg = 40.0f;
    output = pypilot_tack_compute(input, config);
    assert(output.completed);
    assert(!output.override_pilot);
    assert(nearf(output.new_heading_command_deg, 0.0f));

    input.direction = PypilotTackDirection::starboard;
    input.heading_command_deg = -100.0f;
    input.heading_deg = -80.0f;
    output = pypilot_tack_compute(input, config);
    assert(output.override_pilot);
    assert(nearf(output.command_norm, -1.0f));
    assert(nearf(output.new_heading_command_deg, 0.0f));

    input.wind_mode = true;
    input.heading_command_deg = -45.0f;
    input.apparent_wind_direction_deg = -10.0f;
    output = pypilot_tack_compute(input, config);
    assert(output.override_pilot || output.completed);
    assert(nearf(output.new_heading_command_deg, 45.0f));

    return 0;
}
