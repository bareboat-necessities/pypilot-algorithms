#include <cassert>
#include <cmath>
#include <pypilot_algorithms.hpp>

using namespace pypilot_algorithms;

int main() {
    LowPass<float> lp;
    assert(lp.update(10.0f, 0.5f) == 10.0f);
    assert(std::fabs(lp.update(20.0f, 0.5f) - 15.0f) < 0.0001f);

    AngleLowPass<float> ap;
    ap.set(179.0f);
    float v = ap.update(-179.0f, 0.5f);
    assert(v == 180.0f || std::fabs(v + 180.0f) < 0.0001f || std::fabs(v - 180.0f) < 0.0001f);

    Derivative<float> d;
    assert(d.update(10.0f, 1.0f) == 0.0f);
    assert(std::fabs(d.update(13.0f, 0.5f) - 6.0f) < 0.0001f);

    AngleDerivative<float> ad;
    ad.set(179.0f);
    assert(std::fabs(ad.update(-179.0f, 1.0f) - 2.0f) < 0.0001f);

    return 0;
}
