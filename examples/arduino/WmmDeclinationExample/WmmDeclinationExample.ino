#include <Arduino.h>
#include <pypilot_algorithms.hpp>

extern "C" {
#include "../../../extras/wmm_tinier/wmm_core.c"
#include "../../../extras/wmm_tinier/WMM_COF.c"
}

#include <pypilot_algorithms/wmm.hpp>

void setup() {
    Serial.begin(9600);

    float declination_deg = 0.0f;
    const bool ok = pypilot_algorithms::wmm::declination_deg(
        40.7f, -73.9f,
        2026, 6, 18,
        declination_deg);

    if (ok) {
        Serial.print("declination_deg=");
        Serial.println(declination_deg, 4);
    } else {
        Serial.println("declination invalid");
    }
}

void loop() {
}
