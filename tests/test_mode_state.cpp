#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

static bool nearf(float a, float b) { return std::fabs(a - b) < 0.0001f; }

int main() {
    uint32_t mask = pypilot_available_modes(true, true, false, true, true, true);
    assert(pypilot_mode_available(mask, PypilotMode::compass));
    assert(pypilot_mode_available(mask, PypilotMode::gps));
    assert(!pypilot_mode_available(mask, PypilotMode::nav));
    assert(pypilot_mode_available(mask, PypilotMode::wind));
    assert(pypilot_mode_available(mask, PypilotMode::true_wind));

    assert(pypilot_best_mode(PypilotMode::nav, mask) == PypilotMode::gps);
    assert(pypilot_best_mode(PypilotMode::true_wind, pypilot_mode_mask_wind) == PypilotMode::wind);
    assert(pypilot_best_mode(PypilotMode::wind, pypilot_mode_mask_compass) == PypilotMode::compass);

    assert(nearf(pypilot_mode_heading(PypilotMode::compass, 80.0f, 20.0f, 60.0f, 70.0f), 80.0f));
    assert(nearf(pypilot_mode_heading(PypilotMode::gps, 80.0f, 20.0f, 60.0f, 70.0f), 100.0f));
    assert(nearf(pypilot_mode_heading(PypilotMode::nav, 80.0f, 20.0f, 60.0f, 70.0f), 100.0f));
    assert(nearf(pypilot_mode_heading(PypilotMode::wind, 80.0f, 20.0f, 60.0f, 70.0f), -20.0f));
    assert(nearf(pypilot_mode_heading(PypilotMode::true_wind, 80.0f, 20.0f, 60.0f, 70.0f), -10.0f));

    return 0;
}
