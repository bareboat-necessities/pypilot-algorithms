# WMM_Tinier upstream snapshot

Source: https://github.com/DavidArmstrong/WMM_Tinier
Pinned upstream commit: `30268994d2bded9e5168949466db09a1cd00a83f`
Upstream version noted in README: 1.0.3 - October 25, 2025
License: MIT, see `LICENSE` in this directory.

WMM_Tinier is an Arduino-oriented compact C99 implementation of the World Magnetic Model 2025 coefficients for magnetic declination/variation calculation over 2025-2030.

This pypilot-algorithms vendor copy intentionally imports the portable C core files only:

* `wmm.h`
* `wmm.c`
* `WMM_COF.c`

The upstream Arduino class wrapper (`WMM_Tinier.h/.cpp`) is not used directly because it depends on Arduino printing types and is not suitable as the pypilot algorithm API boundary.
