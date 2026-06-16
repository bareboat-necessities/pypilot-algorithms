#include <iostream>
#include <pypilot_algorithms.hpp>

int main() {
    pypilot_algorithms::BasicPilotGains<> gains;
    gains.P = 0.003f;
    gains.D = 0.1f;
    gains.DD = 0.05f;

    pypilot_algorithms::BasicPilotInput<> input;
    input.heading_error_deg = -5.0f;
    input.headingrate_deg_s = 0.4f;
    input.headingraterate_deg_s2 = -0.1f;

    auto out = pypilot_algorithms::compute_basic_pilot(input, gains);
    std::cout << "command_norm=" << out.command_norm << "\n";
    std::cout << "Pgain=" << out.Pgain << "\n";
    std::cout << "Dgain=" << out.Dgain << "\n";
    std::cout << "DDgain=" << out.DDgain << "\n";
    return 0;
}
