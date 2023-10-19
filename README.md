# Changes
M18/M84 supports individual axes(M18 X will only turn off steppers associated with X, if you are running CoreXY, X and Y will be turned off as they are connected)<br>
G28 supports conditional homing (G28 X0 will only home X if X has not been homed yet)<br>
PID Profiles: generate profiles with PID values for different temperatures and load them with a command without restarting<br>
Improved PID tuning algorithm and velocity based PID algorithm (https://github.com/Klipper3d/klipper/pull/5955)<br>
Implemented https://github.com/Klipper3d/klipper/pull/6346 <br>
Implemented https://github.com/Klipper3d/klipper/pull/6246 <br>
Implemented https://github.com/Klipper3d/klipper/pull/6290 <br>
Implemented https://github.com/Klipper3d/klipper/pull/6226 <br>
Implemented https://github.com/Klipper3d/klipper/pull/6239 <br>
Implemented https://github.com/Klipper3d/klipper/pull/6224 <br>
You can now call SAVE_CONFIG without an automatic restart (see docs) <br>
MARK_AS_HOME and MARK_AS_UNHOMED commands (see docs) <br>
max_set_temp option for heaters (see docs) <br>
homing_resting_retract_dist and homing_resting_retract_speed (see docs) <br>
disable_on_error for steppers (see docs) <br>
Filament Switch sensors support Marlins Runout Distance (can also be changed on the fly) <br>
Detection Length for Filament Motion Sensors can be changed on the fly <br>
Implementation of Marlins Cold Extrude<br>
Fixed some stuff in the stepper_enable code<br>
Added support for the run_on_error feature from the LED-Effect plugin (also requires our forked version of LED-Effect)<br>
Added jinja Loop-Controls<br>
Implemented controller_temperature_fan (look at the doc for explanation)<br>
Implemented "curve" control algorithm for temperature_fan (look at the doc for explanation)<br>
Other minor fixes and debug features that do not impact general use<br>

All commands and config options for the features should be documented in the docs, if I forgot something, feel free to open an issue and I will get to it ASAP<br>

Pretty much every normal klipper config is compatible, except for one thing: https://github.com/Klipper3d/klipper/pull/6307<br>



# Original README
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

