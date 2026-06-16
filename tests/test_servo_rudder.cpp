#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    assert(pypilot_rudder_feedback_valid(true, true, 30.0f));
    assert(!pypilot_rudder_feedback_valid(false, true, 30.0f));
    assert(!pypilot_rudder_feedback_valid(true, false, 30.0f));
    assert(!pypilot_rudder_feedback_valid(true, true, 0.0f));
    assert(nearf(pypilot_rudder_position_command(0.5f, 30.0f), 15.0f));
    assert(!pypilot_servo_faulted(0));
    assert(pypilot_servo_faulted(pypilot_servo_overtemp_fault));
    assert(pypilot_servo_saturated_fault(pypilot_servo_saturated));
    assert(!pypilot_servo_allows_command(pypilot_servo_saturated));
    assert(nearf(pypilot_reset_command<float>(), 0.0f));
    return 0;
}
