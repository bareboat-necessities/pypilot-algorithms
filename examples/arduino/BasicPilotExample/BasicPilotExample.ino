#include <Arduino.h>
#include <pypilot_algorithms.hpp>

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pypilot_algorithms::BasicPilotGains<> gains;
  gains.P = 0.003f;
  gains.D = 0.1f;

  pypilot_algorithms::BasicPilotInput<> input;
  input.heading_error_deg = -5.0f;
  input.headingrate_deg_s = 0.4f;

  auto out = pypilot_algorithms::compute_basic_pilot(input, gains);
  Serial.print("command_norm=");
  Serial.println(out.command_norm, 6);
}

void loop() {}
