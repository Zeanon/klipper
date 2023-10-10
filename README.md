# Changes
M18/M84 supports individual axes(M18 X will only turn off steppers associated with X, if you are running CoreXY, X and Y will be turned off as they are connected)
G28 supports conditional homing (G28 X0 will only home X if X has not been homed yet)
PID Profiles: generate profiles with PID values for different temperatures and load them with a command without restarting
Improved PID tuning algorithm and velocity based PID algorithm (https://github.com/Klipper3d/klipper/pull/5955)
Filament Switch sensors support Marlins Runout Distance (can also be changed on the fly) (currently on the runout_test branch)
Detection Length for Filament Motion Sensors can be changed on the fly (currently on the runout_test branch)
Implementation of Marlins Cold Extrude
Fixed some stuff in the stepper_enable code
Added support for the run_on_error feature from the LED-Effect plugin (also requires our forked version of LED-Effect)
Added jinja Loop-Controls
Implemented controller_temperature_fan (look at the doc for explanation)
Implemented "curve" control algorithm for temperature_fan (look at the doc for explanation)
Other minor fixes and debug features that do not impact general use

All commands and config options for the ffeatures should be documented in the docs, if I forgot something, feel free to open an issue and I will get to it ASAP

Pretty much every normal klipper config is compatible, except for one thing: https://github.com/Klipper3d/klipper/pull/6307



Welcome to the Klipper project!

[![Klipper](docs/img/klipper-logo-small.png)](https://www.klipper3d.org/)

https://www.klipper3d.org/

Klipper is a 3d-Printer firmware. It combines the power of a general
purpose computer with one or more micro-controllers. See the
[features document](https://www.klipper3d.org/Features.html) for more
information on why you should use Klipper.

To begin using Klipper start by
[installing](https://www.klipper3d.org/Installation.html) it.

Klipper is Free Software. See the [license](COPYING) or read the
[documentation](https://www.klipper3d.org/Overview.html). We depend on
the generous support from our
[sponsors](https://www.klipper3d.org/Sponsors.html).

