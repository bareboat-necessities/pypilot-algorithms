#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

int main() {
    assert(wrap_360_deg(0.0) == 0.0);
    assert(wrap_360_deg(360.0) == 0.0);
    assert(wrap_360_deg(-1.0) == 359.0);

    assert(wrap_180_deg(181.0) == -179.0);
    assert(wrap_180_deg(-181.0) == 179.0);
    assert(wrap_180_deg(180.0) == 180.0);

    assert(heading_error_deg(10.0, 20.0) == -10.0);
    assert(heading_error_deg(350.0, 10.0) == -20.0);
    assert(heading_error_deg(0.0, 100.0) == -30.0);
    assert(command_error_deg(100.0, 0.0) == 30.0);

    return 0;
}
