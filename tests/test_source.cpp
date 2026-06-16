#include <cassert>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

int main() {
    assert(pypilot_source_can_update(PypilotSource::signalk, PypilotSource::serial));
    assert(!pypilot_source_can_update(PypilotSource::serial, PypilotSource::signalk));
    assert(pypilot_source_can_update(PypilotSource::serial, PypilotSource::signalk, false));
    assert(pypilot_source_can_update(PypilotSource::none, PypilotSource::signalk));
    assert(pypilot_source_can_update(PypilotSource::gps_wind, PypilotSource::water_wind));
    assert(!pypilot_source_can_update(PypilotSource::serial, PypilotSource::water_wind));
    assert(pypilot_source_can_update(PypilotSource::serial, PypilotSource::signalk, 9000002ULL, 1ULL));
    return 0;
}
