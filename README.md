# pypilot-algorithms

Header-only C++11 math and control algorithms extracted for a C++ port of current pypilot.

This project contains pure reusable algorithm code only:

- circular heading/angle math
- low-pass and derivative filters
- gain contribution helpers
- current pypilot-style basic pilot command math
- current pypilot-style wind pilot command math

It deliberately does **not** contain:

- TCP client protocol
- JSON parsing
- servo UART protocol
- data-model storage
- sockets
- Arduino WiFi/Ethernet objects
- IMU hardware drivers
- AHRS/INS implementation

## Public include

Use the same include on Linux and Arduino:

```cpp
#include <pypilot_algorithms.hpp>
```

## Build on Linux

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Example

```cpp
#include <pypilot_algorithms.hpp>

pypilot_algorithms::BasicPilotGains<float> gains;
gains.P = 0.003f;
gains.D = 0.1f;

pypilot_algorithms::BasicPilotInput<float> input;
input.heading_error_deg = -5.0f;
input.headingrate_deg_s = 1.2f;

auto out = pypilot_algorithms::compute_basic_pilot(input, gains);
```

## Scope

This library is the math-heavy base used before implementing higher-level modules such as `pypilot-pilots-cpp` and `pypilot-autopilot-core-cpp`.

## License

GPL-3.0-or-later.
