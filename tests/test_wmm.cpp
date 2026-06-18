#include <cassert>
#include <cmath>
#include <pypilot_algorithms/wmm.hpp>

static bool finite_declination(const pypilot_algorithms::wmm::MagneticResult& r) {
    return r.valid && std::isfinite(r.declination_deg) && r.declination_deg >= -180.0f && r.declination_deg <= 180.0f;
}

int main() {
    pypilot_algorithms::wmm::WorldMagneticModel model;
    assert(model.begin());

    const pypilot_algorithms::wmm::Date date(2026, 6, 18);
    const pypilot_algorithms::wmm::Position points[] = {
        {40.7f,  -73.9f, 0.0f},   // North Atlantic / New York Bight
        {18.4f,  -66.1f, 0.0f},   // Caribbean
        {37.8f, -122.4f, 0.0f},   // Eastern Pacific / San Francisco Bay
        {51.5f,    1.0f, 0.0f},   // North Sea / English Channel
        {36.0f,   14.5f, 0.0f},   // Mediterranean / Malta
        {-34.6f, -58.4f, 0.0f},   // South Atlantic / Rio de la Plata
        {-20.0f,  57.5f, 0.0f},   // Indian Ocean / Mauritius
        {35.7f,  139.7f, 0.0f},   // Western Pacific / Tokyo Bay
        {-36.8f, 174.8f, 0.0f},   // Tasman Sea / Auckland
        {-54.8f, -68.3f, 0.0f}    // Southern Ocean / Tierra del Fuego
    };

    float min_decl = 999.0f;
    float max_decl = -999.0f;
    for (const auto& p : points) {
        const auto r = model.evaluate(p, date);
        assert(finite_declination(r));
        if (r.declination_deg < min_decl) min_decl = r.declination_deg;
        if (r.declination_deg > max_decl) max_decl = r.declination_deg;
    }
    assert(max_decl - min_decl > 1.0f);

    const auto base = model.evaluate({37.8f, -122.4f, 0.0f}, date);
    const auto wrapped = model.evaluate({37.8f, 237.6f, 0.0f}, date);
    assert(finite_declination(base));
    assert(finite_declination(wrapped));
    assert(std::fabs(base.declination_deg - wrapped.declination_deg) < 0.001f);

    float declination = 0.0f;
    assert(model.declination_deg(40.7f, -73.9f, 2026, 6, 18, declination));
    assert(std::isfinite(declination));

    assert(!model.evaluate({91.0f, 0.0f, 0.0f}, date).valid);
    assert(!model.evaluate({0.0f, 0.0f, 0.0f}, {2024, 12, 31}).valid);
    assert(!model.evaluate({0.0f, 0.0f, 0.0f}, {2031, 1, 1}).valid);
    assert(!model.evaluate({0.0f, 0.0f, 0.0f}, {2026, 2, 30}).valid);
    assert(model.evaluate({0.0f, 0.0f, 0.0f}, {2028, 2, 29}).valid);

    return 0;
}
